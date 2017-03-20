
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
 ***** 0.02 5-3-2003
 ***** Added 500 us delay to prevent pegging the CPU
 *****
 ***** 0.01 5-24-2002 -BR
 ***** First working version. Support for joystick disabling for Phase 
 ***** IV operation.
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
#include <baseClient.h>

int demoRunning = 0; 

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
     fprintf(stderr, "Red button pressed.\n");
     demoRunning = 0;
     buttonSetButtons( BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_ON );
     system("/home/bee/src/beeExamples/sendbutton.pl red &");
   }
  if (data->yellow_button_pressed) {
     fprintf(stderr,"Yellow button pressed.\n");
  }
  if (data->blue_button_pressed) {
     fprintf(stderr,"Blue button pressed.\n");
   /*  system("/home/bee/speakit2.pl Can I push your buttons too? &"); */
     system("/home/bee/src/beeExamples/sendbutton.pl blue &");
  }
  if ((data->green_button_pressed) && (demoRunning == 0)) {
     fprintf(stderr,"Green button pressed, disabling the joystick.\n");
     demoRunning = 1;
     buttonSetButtons( BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF );
     joystickDisable(1);
     system("/home/bee/src/beeExamples/sendbutton.pl green &");
  }
 
  if (data->blue_button_changed) {
     fprintf(stderr,"Blue button changed\n");
  }
  if (data->green_button_changed) {
     fprintf(stderr,"Green button changed\n");
  }

  
  return 0;
}


/*------------------------------------------------------------*/
  
void
baseCallback(unsigned long opcode, unsigned long value)
{
  
  switch (opcode) {
  
   /* error conditions */
  case BASE_translateError:
  case BASE_rotateError:
  case BASE_batteryHigh:
  case BASE_batteryLow:
    fprintf(stderr, "%s:%6d:%s() - error code = %d\n",
            __FILE__, __LINE__, __FUNCTION__, opcode);
    break;
 
  case BASE_joystickDisable:
    fprintf(stderr, "%s:%6d:%s() - BASE_joystickDisable = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, opcode);
    break;

  case BASE_rotateHalt:
  /*  fprintf(stderr, "%s:%6d:%s() - BASE_rotateHalt\n",
            __FILE__, __LINE__, __FUNCTION__); */
    break;

  case BASE_translateHalt:
  /*  fprintf(stderr, "%s:%6d:%s() - BASE_translateHalt\n",
            __FILE__, __LINE__, __FUNCTION__); */
    break;
  
   /* commands that return values, message is used when returning value too */
  case BASE_batteryCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_batteryCurrent = %5.1f\n",
            __FILE__, __LINE__, __FUNCTION__, (float)value/10.0);
    break;
  
  case BASE_batteryVoltage:
    fprintf(stderr, "%s:%6d:%s() - BASE_batteryVoltage = %5.1f\n",
            __FILE__, __LINE__, __FUNCTION__, (float)value/10.0);
    break;
    
  case BASE_rotateCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateCurrent = %d\n",
            __FILE__, __LINE__, __FUNCTION__, value);


  case BASE_rotateWhere:
   /* fprintf(stderr, "%s:%6d:%s() - BASE_rotateWhere = %d\n",
            __FILE__, __LINE__, __FUNCTION__, value); */
    break;
  
  case BASE_translateCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateCurrent = %d\n",
            __FILE__, __LINE__, __FUNCTION__, value);
    break;
  
  case BASE_translateWhere:
 /*   fprintf(stderr, "%s:%6d:%s() - BASE_translateWhere = %d\n",  
            __FILE__, __LINE__, __FUNCTION__, value); */
    break;
    
  
  case BASE_indexReport:
 /*   fprintf(stderr, "%s:%6d:%s() - BASE_indexReport = %d\n",
            __FILE__, __LINE__, __FUNCTION__, value); */
    break;
  
  default:
    fprintf(stderr, "%s:%6d:%s() - unexpected opcode = %d, value= %d\n",
            __FILE__, __LINE__, __FUNCTION__, opcode, value);
  }
  
  return;
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
  registerBaseClient();

  initClient( "buttonCall4", commShutdown); /* close function called if */
                                              /* the server dies          */

  buttonConnect( 1 );  /* 1 -> wait until connection has been established */
  
  baseConnect(1); 
  
  RaiInit();                        /* init (but not start) scheduler */

  catchInterrupts();

  initClientModules();                  /* set up Rai modules to do         */
                                        /* communication for you            */

  /* whenever user hits CTRL-C */
  signal( SIGINT, &ctrlcShutdown );

  /* we wanna know when the buttons' status change ... */
  registerButtonStatusCallback( myButtonStatusCallback );
  
  buttonSubscribeStatus( 1 );

  /* Base interface stuff */
  registerBaseCallback(baseCallback);
 /*  registerStatusCallback(statusCallback); */


  /* and flash some buttons ... */
/*  buttonStartCuteThing(); */

 /* Turn E-stop and red buttons on */
  buttonSetButtons( BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_OFF, BUTTON_LIGHT_STATUS_ON, BUTTON_LIGHT_STATUS_ON );
  
  while ( 1 ) {
      
    TCX_waiting_time.tv_sec  = 0;
    TCX_waiting_time.tv_usec = 500;
    tcxRecvLoop((void *) &TCX_waiting_time);
    
  }

  RaiStart();

  return 0;
}

