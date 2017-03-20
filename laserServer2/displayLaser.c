#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <X11/Xlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include "EZX11.h"

#define TCX_define_variables 
#define DEFINE_REPLY_HANDLERS

#include "tcx.h"
#include "tcxP.h"

#include "LASER_SERVER-messages.h"

#define LENGTH_OF_MODULE_NAMES 128
#define NUMBER_OF_TCX_MODULES  2

#define LASERDISPLAY_MODULE    0
#define LASER_SERVER_MODULE    1

#define TRUE 1
#define FALSE 0

#define FILENAMELENGTH 80

#define ROB_RADIUS 30.0
#define ROB_HEIGHT 120.0

#define NUMBER_OF_SONARS       24
#define NUMBER_OF_LASER_BEAMS  361
#define LASER_MAX_RANGE        800.0

#define MAX_NAME_SIZE           80

#define DEG_90  M_PI_2
#define DEG_180 M_PI
#define DEG_270 (M_PI + M_PI_2)
#define DEG_360 (M_PI + M_PI)

#define WINDOW_SIZE             200
#define LASER_OFFSET            11.5
#define LASER_START_ANGLE       DEG_270

#define ROBOT_RADIUS            26.0     /* in cm */

#define TIME_TRY_TO_CONNECT     2

typedef unsigned int bool;

typedef struct {
  int fValues;
  float flaser[NUMBER_OF_LASER_BEAMS];
  int rValues;
  float rlaser[NUMBER_OF_LASER_BEAMS];
} laserStatusType;

struct timeval TCX_waiting_time = {0, 0};

EZXW_p mainwin;

float VISIBLE_DIST   = 500.0;
int UPDATE_INTERVAL  = 3;
int onelaser         = FALSE;

int window_size      = WINDOW_SIZE;
int use_robot_name   = FALSE;

char robotName[MAX_NAME_SIZE];

laserStatusType laserStatus;

bool tcx_initialized = FALSE;

extern laserStatusType laserStatus;
extern int UPDATE_INTERVAL;

static struct timeval lcct_LServer  = {0, 0};
static struct timeval curtime       = {0, 0};

int LServerConnected     = FALSE;

extern int use_robot_name;

/* initialize module names */

extern char robotName[MAX_NAME_SIZE];

char moduleName[NUMBER_OF_TCX_MODULES][LENGTH_OF_MODULE_NAMES];

extern void tcxRegisterCloseHnd(void (*closeHnd)());


/*****************************************************************************
 *****************************************************************************/

float
Deg2Rad(float x)
{
  return x * 0.017453293;
}

float
Rad2Deg(float x)
{
  return x * 57.29578;
}

EZXW_p 
CreateNewWindow()
{
  EZXW_p win;
  EZX_NoMotionEvents();
  if (onelaser)
    win = EZX_MakeWindow("LASERDISPLAY",window_size, window_size/2,"");
  else
    win = EZX_MakeWindow("LASERDISPLAY",window_size, window_size,"");
  EZX_Flush();
  return win;
}

float
correctX( float pos )
{
  return(pos);
}

float
correctY( float pos )
{
  return(window_size-pos);
}

float
ScrPosX( float middle, float length, float heading )
{
  return(correctX(middle+length*cos(M_PI_2-Deg2Rad(heading))));
}

float
ScrPosY( float middle, float length, float heading )
{
  return(correctY(middle+length*sin(M_PI_2-Deg2Rad(heading))));
}


/*****************************************************************************
 *****************************************************************************/

void
DrawRobot( EZXW_p eWin )
{
  int middle   = window_size / 2;
  float factor = window_size / VISIBLE_DIST;
  
  EZX_SetColor( C_YELLOW);
  EZX_FillCircle( eWin, middle, middle, factor * ROBOT_RADIUS );
  EZX_SetColor( C_BLACK);
  EZX_DrawCircle( eWin, middle, middle, factor * ROBOT_RADIUS );
  EZX_DrawLine( eWin, middle, middle, 
		ScrPosX( middle, factor * ROBOT_RADIUS, 0 ),
		ScrPosY( middle, factor * ROBOT_RADIUS, 0 ) );
}

void
displayBeams( void )
{
  float adf = 180.0 / laserStatus.fValues;
  float adr = 180.0 / laserStatus.fValues;
  int i;
  int middle   = window_size / 2;
  float factor = window_size / VISIBLE_DIST;
  int psize    = factor * 4;
  if (psize<1) psize=1;
  EZX_ClearWindow(mainwin);
  EZX_SetColor( C_WHITE);

  for (i=0;i<laserStatus.fValues;i++)
   EZX_FillCircle( mainwin, 
		   ScrPosX(middle,factor*laserStatus.flaser[i],90-i*adf),
		   ScrPosY(middle,factor*laserStatus.flaser[i],90-i*adf), 
		   psize);
  if (!onelaser)
    for (i=0;i<laserStatus.rValues;i++)
      EZX_FillCircle( mainwin, 
		      ScrPosX(middle,factor*laserStatus.flaser[i],270-i*adr),
		      ScrPosY(middle,factor*laserStatus.flaser[i],270-i*adr), 
		      psize);
  DrawRobot( mainwin );
}


/*****************************************************
  REPLY
*****************************************************/

void
LASER_SERVER_sweep_reply_handler   ( TCX_REF_PTR ref,
				     LASER_SERVER_sweep_reply_ptr sweep);
     
/*****************************************************************************
 *****************************************************************************/

void
initializeModuleNames( char *robotName ){
  {
    if (!use_robot_name) 
      robotName = NULL;

    tcxSetModuleName( TCX_LASER_SERVER_MODULE_NAME, robotName, 
		      moduleName[LASER_SERVER_MODULE]);
    tcxSetModuleName( "LASERDISPLAY", robotName, 
		      moduleName[LASERDISPLAY_MODULE]);
  }
}

void 
TCX_close_handler ( char *name, TCX_MODULE_PTR module)
{
  if ( !strcmp(name, TCX_LASER_SERVER_MODULE_NAME ) ) { 
    LServerConnected=FALSE;
    fprintf( stderr, "LASERDISPLAY: try to connect to LASERSERVER ...\n" );
  } else if ( !strcmp(name, SERVER_NAME )) {
    fprintf( stderr, "TCX died, so will I...\n" );
    exit(1);
  }
}

void
init_tcx()
{
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LASER_SERVER_messages,
  };

  fprintf(stderr, " start!\n" );

  fprintf(stderr, "  set module-names(%s) ...", 
	  use_robot_name ? robotName:"" );
  initializeModuleNames(robotName);
  fprintf(stderr, " ok\n" );

  fprintf(stderr, "  connecting to TCX ...\n");
  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "  TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }
  tcxInitialize(moduleName[LASERDISPLAY_MODULE], (void *) tcxMachine);

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(LASER_SERVER_reply_handler_array,
		      sizeof(LASER_SERVER_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterCloseHnd( TCX_close_handler);
  
  tcx_initialized = TRUE;  
}

void 
check_tcx_connections( void )
{ 
  LASER_SERVER_register_auto_update_type ldata;
  gettimeofday(&curtime, NULL);
  if (!LServerConnected) {
    if ( !( curtime.tv_sec < lcct_LServer.tv_sec + TIME_TRY_TO_CONNECT   || 
	    ( curtime.tv_sec == lcct_LServer.tv_sec + TIME_TRY_TO_CONNECT   &&
	      curtime.tv_usec < lcct_LServer.tv_usec  ) ) ) {
      lcct_LServer.tv_sec  = curtime.tv_sec;
      lcct_LServer.tv_usec = curtime.tv_usec;
      LASER_SERVER = tcxConnectOptional( moduleName[LASER_SERVER_MODULE] );
      if (LASER_SERVER != NULL){
	LServerConnected = TRUE;	
	ldata.sweep0 = UPDATE_INTERVAL;
	ldata.sweep1 = UPDATE_INTERVAL;
	fprintf( stderr, "LASERDISPLAY: connected to LASER_SERVER\n" );
	fprintf( stderr, "LASERDISPLAY: send register_auto (%d,%d)\n",
		 ldata.sweep0, ldata.sweep1 );
	tcxSendMsg(LASER_SERVER, "LASER_SERVER_register_auto_update", &ldata);
      }
    }
  }
}

/**********************************************************************
 **********************************************************************
 *
 *  REPLY handlers
 *
 **********************************************************************
 **********************************************************************/

/**********************************************************************
 **********************************************************************
 *
 *  LASER handlers
 *
 **********************************************************************
 **********************************************************************/

void
LASER_SERVER_sweep_reply_handler( TCX_REF_PTR                  ref,
				  LASER_SERVER_sweep_reply_ptr sweep)
{
  int i;
#ifdef USER_debug
  fprintf(stderr, "TCX: Received a LASER_SERVER_sweep_reply message.\n");
#endif
  if (sweep->numLaser==0) {
    for (i = 0; i < sweep->numberLasers; i++){
      if (sweep->value[i] < 0.0)
	laserStatus.flaser[i] = 0.0;
      else if (sweep->value[i] >= LASER_MAX_RANGE) 
	laserStatus.flaser[i] = LASER_MAX_RANGE;
      else
	laserStatus.flaser[i] = sweep->value[i];
    }
    laserStatus.fValues = sweep->numberLasers;
  }
  if (sweep->numLaser==1) {
    for (i = 0; i < sweep->numberLasers; i++){
      if (sweep->value[i] < 0.0)
	laserStatus.rlaser[i] = 0.0;
      else if (sweep->value[i] >= LASER_MAX_RANGE) 
	laserStatus.rlaser[i] = LASER_MAX_RANGE;
      else
	laserStatus.rlaser[i] = sweep->value[i];
    }
    laserStatus.rValues = sweep->numberLasers;
  }
  displayBeams();
  tcxFree("LASER_SERVER_sweep_reply", sweep); 
}

/*****************************************************************************
 *****************************************************************************/

int
main( int argc, char** argv)
{
  int i;

  fprintf(stderr, "check arguments ..." );
  for (i=1; i<argc; i++) {
    if (!strcmp(argv[i],"-winsize") && (argc>i+1))
      window_size = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-range") && (argc>i+1))
      VISIBLE_DIST = (float) atoi(argv[++i]);
    else if (!strcmp(argv[i],"-update") && (argc>i+1))
      UPDATE_INTERVAL = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-onelaser"))
      onelaser = TRUE;
    else if (!strcmp(argv[i],"-robot") && (argc>i+1)) {
      use_robot_name = TRUE;
      strncpy( robotName, argv[++i], MAX_NAME_SIZE );
    } else if (!strcmp(argv[i],"-help")) {
      fprintf(stderr, " ok\n\n --------- HELP ---------\n\n" );
      fprintf(stderr, " -robot NAME   :  set robot name (for tcx)\n" );
      fprintf(stderr, " -winsize SIZE :  set window-size\n" );
      fprintf(stderr, " -range SIZE   :  set max. visible distance\n" );
      fprintf(stderr, " -onelaser     :  display only one laser\n\n" );
      exit(0);
    } else {
      fprintf(stderr, " error\n" );
      fprintf(stderr, "usage: %s [-robot ROBOT] [-winsize SIZE] [-range SIZE] [-onelaser] [-help]\n", argv[0] ); 
      exit(0);
    }
  }
  fprintf(stderr, " ok\n" );

  for (i=0; i<NUMBER_OF_LASER_BEAMS; i++) {
    laserStatus.flaser[i]=0.0; 
    laserStatus.rlaser[i]=0.0; 
  }

  fprintf(stderr, "initialize tcx ..." );
  init_tcx();
  fprintf(stderr, " ok\n" );

  mainwin = CreateNewWindow();

  TCX_waiting_time.tv_sec=0;
  TCX_waiting_time.tv_usec=0;
  while (TRUE) {
    EZX_Flush();
    tcxRecvLoop(&TCX_waiting_time);
    check_tcx_connections();
    usleep(10000);
  }
}
