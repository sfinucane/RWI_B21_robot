
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.4-special (released Januar 02, 1998)
 *****                  this is not an official BeeSoft version
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***** This program executes one of three demo programs.
 ***** The red button terminates the running demo.
 *****
 ***** 0.4 5-3-2003
 ***** Added 500 us delay to TCX loop to prevent pegging the CPU.
 *****
 ***** 0.3 7-16-2001
 ***** Fixed multiple stop problem
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdlib.h>
#include <tcx.h>
#include <tcxP.h>
#include <rai.h>
#include <raiClient.h>
#include <bUtils.h>
#include <signal.h>
#include <buttonClient.h>
#include <Common.h>
/* #include "../baseServer/base.h" */
/* #include "/home/brudy/bee/src/baseServer/base.h" */

int demoRunning;
demoRunning = 0; 

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void commShutdown(char *name, TCX_MODULE_PTR module)
{

#if ( defined(BUTTON_DEBUG) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s(%s): %s died. \n", 
	   __FILE__, __FUNCTION__, name );

  RaiShutdown();

  exit( -1 );

}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
void ctrlcShutdown() 
{
  commShutdown("Ctrl-C",NULL);
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int myButtonStatusCallback ( buttonStatusType *data ) {

  if (((data->red_button_pressed) || (data->red_button_changed)) && (demoRunning == 1)) {
/*     fprintf(stderr,"Red button pressed, shutting down %s\n", demoRunning); */
     fprintf(stderr,"Red button pressed, shutting down demo\n");
     system("/home/bee/src/beeExamples/wanderstop2.sh");
     /* Just in case the robot was moving when the button was pressed */
     rotateHalt();
     translateHalt();
     demoRunning = 0;    
     buttonSetButtons( BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_ON );
   }
  if (data->yellow_button_pressed) {
     fprintf(stderr,"Yellow button pressed.\n");
  }
/*  if ((data->blue_button_pressed) && (demoRunning == 0)) { */
  if (data->blue_button_pressed) {
     fprintf(stderr,"Blue button pressed, starting dance...\n");
 /*    demoRunning = 1; */
 /*     buttonSetButtons( BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF ); */
/*     joystickDisable(1); */
    system("/home/bee/speakit2.pl Can I push your buttons too? &");
  }
  if ((data->green_button_pressed) && (demoRunning == 0)) {
     fprintf(stderr,"Green button pressed, starting wander2...\n");
     demoRunning = 1;
     buttonSetButtons( BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF );
/*     joystickDisable(1); */
     system("/home/bee/src/beeExamples/wanderstart2.sh");
  }
 
  if (data->blue_button_changed) {
     fprintf(stderr,"Blue button changed\n");
  }
  if (data->green_button_changed) {
     fprintf(stderr,"Green button changed\n");
  }

  
  return 0;
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {

  struct bParamList * params = NULL;
  struct timeval TCX_waiting_time = {0, 0};

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  buttonRegister();

  initClient( "buttonCall2", commShutdown); /* close function called if */
                                              /* the server dies          */

  buttonConnect( 1 );  /* 1 -> wait until connection has been established */
  
/*  baseConnect(1); */
  
  RaiInit();                        /* init (but not start) scheduler */

  catchInterrupts();

  initClientModules();                  /* set up Rai modules to do         */
                                        /* communication for you            */

  /* whenever user hits CTRL-C */
  signal( SIGINT, &ctrlcShutdown );

  /* we wanna know when the buttons' status change ... */
  registerButtonStatusCallback( myButtonStatusCallback );
  
  buttonSubscribeStatus( 1 );

  /* and flash some buttons ... */
/*  buttonStartCuteThing(); */

 /* Turn E-stop and red buttons on */
  buttonSetButtons( BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_ON );
  
  while ( 1 ) {
      
    TCX_waiting_time.tv_sec  = 0;
   /* TCX_waiting_time.tv_usec = 0; */
    TCX_waiting_time.tv_usec = 500;
    tcxRecvLoop((void *) &TCX_waiting_time);
    
  }

  RaiStart();

  return 0;
}

