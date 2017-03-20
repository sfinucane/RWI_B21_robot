/*
 * $Id: mainlaser.c,v 1.1 2002/09/14 16:33:01 rstone Exp $
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <rai.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include <bUtils.h>

#include <Common.h>
#include "devUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "handlers.h"
#include "io.h"
#include "mainlaser.h"
#include "LASER_SERVER-messages.h"
#include "laserHandlers.h"
#include "librobot.h"

int robotType = B21_ROBOT;

struct bParamList * bParamList = NULL;

int ROBOT_BACKGROUND = 0;
int ROBOT[2];
int INIT_BUTTON = 0;
extern int listen_for_tcx_events;

int display;
int status = 0;

int laserDef[2];
const char *laserConfig[2];
const char *laserType[2];
const char *laserHost[2];
const char *laserDev[2];
const char *laserBps[2];


int block_wait_with_serial_ports(struct timeval *timeout, int tcx_initialized,
				 int X_initialized);

/* ---------------------------------------------------------
 *
 * is called when connecting modules die
 *
 * --------------------------------------------------------*/
void commShutdown( )
{

#if TOTAL_debug
  fprintf(stdout,"\n--->commShutdown\n");
  fflush( stdout );
#endif

  fprintf( stderr, "%s: TCX Server died. Exiting.\n", __FILE__ );
  exit(-1);

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void init_graphics(int display)
{
  int i = 0;
  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			    "9x15", "10x20", "12x24", 
			    "lucidasans-bold-18"};


#if TOTAL_debug
  fprintf(stdout,"\n--->init_graphics\n");
  fflush( stdout );
#endif

  for (i = 0; i != NUMBER_LASERS; i++){
    LaserSensors[0].angles[i] = -90.0 + (((float)i) * 180.0 /
				      ((float) (NUMBER_LASERS-1)));
    LaserSensors[0].display_values[i] =
      LaserSensors[0].values[i] = MAX_LASER_RANGE * 0.4;
    LaserSensors[1].angles[i] = 90.0 + (((float)i) * 180.0 /
				      ((float) (NUMBER_LASERS-1)));
    LaserSensors[1].display_values[i] =
      LaserSensors[1].values[i] = MAX_LASER_RANGE * 0.6;
  }

  for (i = 0;i != NUMBER_OF_LASERS; i++)
    LaserSensors[i].defined = 0;

  G_set_display(display);

  G_initialize_fonts(7, myfonts);
  G_initialize_graphics("LASER", 70.0, 10.0, C_STEELBLUE4);
  G_set_matrix_display_style(1);
  
  
  /******** ROBOT_BACKGROUND **************************************/
  {
    int switch_num                      = 1;
    static float switch_pos[]           = {0.0, 3.9, 6.0, 9.9};
    static char *switch_texts[]         = {""};
    static int switch_fonts[]           = {2};
    static int switch_background_color[]= {C_GREY70};
    static int switch_frame_color[]     = {C_GREY40};
    static int switch_text_color[]      = {NO_COLOR};
    
    ROBOT_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						    switch_texts,
						    switch_background_color,
						    switch_frame_color, 
						    switch_text_color,
						    switch_fonts);
    
  }
  
  /******** ROBOT *************************************/
  {
    static float pos_r[]                 = {0.0, 3.9, 6.0, 9.9};
    static char *text_r                  = "ROBOT";
    static int robot_font                = 4;
    static int colors_r[]                = {NO_COLOR, NO_COLOR, C_GREY90,
					      C_BLACK, C_TURQUOISE4, C_GREY40,
					      NO_COLOR, NO_COLOR};
    
    for (i = 0;i != NUMBER_OF_LASERS; i++)
      ROBOT[i] =  G_create_robot_object(pos_r, text_r, 
					-(MAX_LASER_RANGE) * 1.05,
					(MAX_LASER_RANGE) *1.05,
					-(MAX_LASER_RANGE) *1.05,
					(MAX_LASER_RANGE) *1.05,
					0.0, 0.0, 90.0, 
					ROBOT_RADIUS,
					NUMBER_LASERS,
					MAX_LASER_RANGE,
					LaserSensors[i].display_values,
					LaserSensors[i].angles,
					colors_r, robot_font);
  }    
  /******** INIT_BUTTON **************************************/
  {
    
    int switch_num                      = 2;
    static float switch_pos[]           = {0.0, 3.9, 5.5, 5.9};
    static char *switch_texts[]         = 
      {"initialize laser", "initialize laser"};
    static int switch_fonts[]           = {2,2};
    static int switch_background_color[]= {C_GREY70, C_RED};
    static int switch_frame_color[]     = {C_GREY40, C_GREY40};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
    INIT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						switch_texts,
						switch_background_color,
						switch_frame_color, 
						switch_text_color,
						switch_fonts);
  } 
  
  G_display_all();

}

/************************************************************************
 *   FUNCTION:     tests mouse and executes whatever we'd like
 *   RETURN-VALUE: 1, if meaningful mouse event found, 0 otherwise
 ************************************************************************/
int test_mouse(int display) {

  G_mouse_ptr mouse_events;
  int num_mouse_events = 0, button = 0;
  float mouse_x = 0.0, mouse_y = 0.0;
  int  number = 0;
  int return_value = 0;
  int mouse_event_detected = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->test_mouse\n");
  fflush( stdout );
#endif


  if (display){
    /****************** CHECK FOR MOUSE EVENT *******************/

    mouse_event_detected = G_test_mouse(0);
    if (mouse_event_detected){


      mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				      &button, &num_mouse_events);
      
      if (mouse_event_detected == 1){ /* reglar mouse press */

	return_value = 1;
	
	
	/****************** EVALUATE MOUSE EVENT *******************/

	/* ******* INIT_BUTTON ******** */
	
	if (G_mouse_event_at(INIT_BUTTON, mouse_events, &number)){ 
	}	  

	/********* JUST UPDATE DISPLAY *********/
	
	else if (button == RIGHT_BUTTON){
	  G_display_all();
	}
	
      }
    }
  }
  return return_value;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int displayParameters() {

#if TOTAL_debug
  fprintf(stdout,"\n--->displayParameters\n");
  fflush( stdout );
#endif

  fprintf(stderr, "******** BeeSoft.ini settings **********\n");
  fprintf(stderr, "-display ............... = %d\n", display );
  fprintf(stderr, "-status (from baseserver)= %d\n", status );
  fprintf(stderr, "front laser? ........... = %d\n", laserDef[0] );
  fprintf(stderr, "front laserConfig ...... = [%s.laser_front]\n", laserConfig[0] );
  fprintf(stderr, "front laserType ........ = %s\n", laserType[0] );
  fprintf(stderr, "front laserHost ........ = %s\n", laserHost[0] );
  fprintf(stderr, "front laserDev ......... = %s\n", laserDev[0] );
  fprintf(stderr, "front laserBps ......... = %s\n", laserBps[0] );
  fprintf(stderr, "rear  laser? ........... = %d\n", laserDef[1] );
  fprintf(stderr, "rear  laserConfig ...... = [%s.laser_rear]\n", laserConfig[1] );
  fprintf(stderr, "rear  laserType ........ = %s\n", laserType[1] );
  fprintf(stderr, "rear  laserHost ........ = %s\n", laserHost[1] );
  fprintf(stderr, "rear  laserDev ......... = %s\n", laserDev[1] );
  fprintf(stderr, "rear  laserBps ......... = %s\n", laserBps[1] );
  fprintf(stderr, "*****************************\n");

  
  return 0;

}

int main( int argc, char *argv[] ) {

  struct timeval currentCycleTime, lastCycleTime = {0, 0};

  struct bParamList * bParamList = NULL;
  struct timeval TCX_waiting_time = {0, 0};
  char dummy[256];
  int i = 0;
  char* robotName = NULL;
  char* frontDevice = NULL;
  char* rearDevice = NULL;
  int pioneer_test = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->main\n");
  fflush( stdout );
#endif

  status =0;
  display =0;

  /* add some parameter files */
  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);


  for (i=1; i<argc; i++) {
    if ((strcasecmp(argv[i],"-display")==0)) {
      display = 1;
    }
    else if ((strcasecmp(argv[i],"-status")==0)) {
      status = 1;
    }
    else if ((strcasecmp(argv[i],"-robot")==0)){
      if (i + 1 < argc && argv[i+1][0]!='-'){
        i++;
        robotName = argv[i];
      }
      else {
        fprintf(stderr, "ERROR: robot name must follow keyword robot.\n");
        exit(0);
      }
    }
    else if(strcasecmp(argv[i],"-b18")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "B18");
      robotType = B18_ROBOT;
      pioneer_test = 0;
    }
    else if(strcasecmp(argv[i],"-urban")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "URBAN");
      robotType = URBAN_ROBOT;
      pioneer_test = 0;
    }
    else if(strcasecmp(argv[i],"-pioneer2")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_II");
      robotType = PIONEER_II;
      pioneer_test = 1;
    }
    else if(strcasecmp(argv[i],"-pioneer")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_I");
      robotType = PIONEER_I;
      pioneer_test = 1;
    }
    else if(strcasecmp(argv[i],"-pioneerf")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_IF");
      robotType = PIONEER_IF;
      pioneer_test = 1;
    }
    else if(strcasecmp(argv[i],"-pioneerat")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_AT");
      robotType = PIONEER_ATRV;
      pioneer_test = 1;
    }
    else if(strcasecmp(argv[i],"-scout")==0) {
      /* add some parameter files */
      robotType = Scout;
      pioneer_test = 0;
      frontDevice = "/dev/ttyS1";
    }
    else if ((strcasecmp(argv[i],"-frontDev")==0)){
      if (i < argc - 1) {
        i++;
        frontDevice = argv[i];
	fprintf(stderr, "Set front device to %s.\n", frontDevice);
      }
      else {
        fprintf(stderr, "ERROR: device name must follow keyword -frontDev.\n");
        exit(0);
      }
    }
    else if ((strcasecmp(argv[i],"-rearDev")==0)){
      if (i < argc - 1) {
        i++;
        rearDevice = argv[i];
      }
      else {
        fprintf(stderr, "ERROR: device name must follow keyword -rearDev.\n");
        exit(0);
      }
    }
    else {
      fprintf(stderr, "Usage: %s [-robot name] [-pioneer] [-pioneer2] [-pioneerf] [-pioneerat] [-urban] [-b18] [-display] [-status] [-frontDev dev]  [-rearDev dev]\n", argv[0]);
      exit(0);
    }
  }

  if (pioneer_test)
    /* add some parameter files */
    bParamList = bParametersAddEntry(bParamList, "robot", "pioneer","yes");
  else
    bParamList = bParametersAddEntry(bParamList, "robot", "pioneer","no");

  /*
    if (robotName != NULL) {
    bParamList = bParametersAddEntry(bParamList, "robot", "name", robotName);
  }
  */
  
  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);  
  
  tcxMachine = (char*)bParametersGetParam(bParamList, "", "TCXHOST");

  /* find out the current laser config in the robot 
     section should be one of { none, SICK }        */
  if (pioneer_test)
    laserConfig[0] = bParametersGetParam( bParamList, "robot", "laser" );
  else {
  laserConfig[0] = bParametersGetParam( bParamList, "robot", "laser_front" );
  laserConfig[1] = bParametersGetParam( bParamList, "robot", "laser_rear" );
  }
  


  /* Start the laser(s). Settings depend on robotType which is set
   * to B21 per default. For specific values see io.c. */
  start_laser( frontDevice, rearDevice);

  init_graphics(display);
    
  LaserInitializeTCX( robotName);

  for (i = 0;i !=  NUMBER_OF_LASERS; i++)
    if (!laserConfig[i] || !strcasecmp( laserConfig[i], "none" ))
      laserDef[i] = 0;
    else
      laserDef[i] = 1;

  if (!laserDef[0] && !laserDef[1]) {
    fprintf( stderr,"Your config in beeSoft.ini says, that you\n" );
    fprintf( stderr,"don't have a laser. What am I supposed to do?\n" );
    exit(-1);
  }

  for (i = 0;i != NUMBER_OF_LASERS; i++)
    if (laserDef[i]){
      /* get variables from the section `[<laserConfig>.laser]' */
      if (i == 0)
	sprintf( dummy, "%s.%s", laserConfig[0], "laser_front" );
      else
	sprintf( dummy, "%s.%s", laserConfig[1], "laser_rear" );
      laserType[i] = bParametersGetParam( bParamList, dummy, "type" );
      laserHost[i] = bParametersGetParam( bParamList, dummy, "host" );
      laserDev[i] = bParametersGetParam( bParamList, dummy, "dev" );
      laserBps[i] = bParametersGetParam( bParamList, dummy, "bps" );
      
      
      if (!laserType[i] || !laserHost[i] || !laserDev[i] || !laserBps[i]){
	fprintf( stderr,"Couldn't parse beeSoft.ini properly.\n");
	exit(-1);
      }
    }

  displayParameters();

  /*setupLaser();*/
  if (status) connect_to_BaseServer();  

  listen_for_tcx_events = 1;

  for (;;) {
    
    for (i = 0; i !=NUMBER_OF_LASERS ; i++){
      if ( LaserSensors[i].defined ) {
	if ( (i == 0 && n_auto_sweep0_update_modules ) ||
	     (i == 1 && n_auto_sweep1_update_modules ))
	  send_automatic_sweep_update( i, LaserSensors[i].values );
	if (i == 0)
	   G_display_switch(ROBOT_BACKGROUND, 0);
	G_display_robot( ROBOT[i], 0.0, 0.0, 90.0, NUMBER_LASERS,
			 &(LaserSensors[i].display_values[0]));
	LaserSensors[i].defined = 0;
      }
    }

    if (display)
      EZX_Flush();

    TCX_waiting_time.tv_sec  = 1;
    TCX_waiting_time.tv_usec = 1;
    block_wait_with_serial_ports(&TCX_waiting_time, 1, display);

    gettimeofday ( & currentCycleTime, (struct timezone *) NULL );

    /* Deal with idiocy, and ignore >1 second time gaps */
    if (   ( currentCycleTime.tv_sec >= lastCycleTime.tv_sec      )
	&& ( currentCycleTime.tv_sec  - lastCycleTime.tv_sec <= 1 ) )
    {
      if ( currentCycleTime.tv_sec != lastCycleTime.tv_sec )
      {
	lastCycleTime.tv_sec += 1;
	lastCycleTime.tv_usec -= 1000000;
      }
	/* These two should be equal now... */
      if ( currentCycleTime.tv_sec != lastCycleTime.tv_sec )
      {
	fprintf(stderr, "[mainLaser.c:main()] Delay-time code is confused.\n");
      }
      else
      {
	if ( 200001 + lastCycleTime.tv_usec - currentCycleTime.tv_usec > 0 )
	  usleep ( 200001 + lastCycleTime.tv_usec - currentCycleTime.tv_usec );
      }
    }
    gettimeofday ( & lastCycleTime, (struct timezone *) NULL );

    ProcessDevices();

    /*fprintf(stderr, "-");*/
    LaserPing(&frontLaserDevice);
    LaserPing(&rearLaserDevice);
    if (status) connect_to_BaseServer();
    test_mouse( display );
    
  }

}
