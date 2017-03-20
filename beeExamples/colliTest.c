

/*===========================================*/
/*===========================================*/
/* colliTest.c */
/*===========================================*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <baseClient.h>
#include <colliClient.h>
#include <rai.h>
/*===========================================*/
int gotOrigin = FALSE;
int statusCount = 0;
int
tactileCallback(tactileType ** newTactile)
{
     int i;
     int j;
     /* Stop the robot if any tactile sensor */
     /* is pushed against. */
     for (i=0; i<bRobot.tactile_rows; i++) {
          for (j=0; j<bRobot.tactile_cols[i];j++) {
               if (tactiles[i][j].value) {
                    printf("**** Touched on %d, %d\n",i,j);
                    bSetVel(0.0, 0.0);
                    return;
               }
          }
     }
     return;
}
/*===========================================*/
/* This routine issues the colliApproachAbsolute */
/* command the first time it is polled after the */
/* origin is established. */
void
demoPoll(RaiModule * demoModule)
{
     static time = 0;
     if (gotOrigin) {
          if (time == 0) {
            fprintf(stderr, "Setting colliApproachAbsolute\n");
            colliApproachAbsolute(-100.0, 0.0, 50.0, 1);
          }
          if (time>300) {
               RaiShutdown( );
          }
          time++;
     }
     return;
}
/*===========================================*/
void
statusCallback(statusReportType * newStatus)
{
     statusCount++;
     if ((!gotOrigin) && (statusCount < 10)) {
          bSetPosition(0.0, 0.0, 0.0);
          gotOrigin = TRUE;
     }
}
/*===========================================*/
void
createDemoModule( )
{
     RaiModule* demoModule;
     int ii;
     printf ("Setting up robot program\n");
/* Set robot's heading; units are radians*512/pi. */
     loadHeading(0);
/* Set the robot's initial position.*/
/* Units are encoder counts/256. */
     loadPosition(0x80008000);
/* Set up the status reporting timing; */
/* Units are seconds*256. */
     statusReportPeriod(100*256/1000);
     registerStatusCallback (statusCallback);
     registerTactileCallback (tactileCallback);
/* Set the robot's initial velocities, in cm/sec.  */
     colliSetVelocity(20.0, M_PI/2.0);
/* Set the robot's initial acceleration values, */
/* in centimeters per second squared. */
     colliSetAcceleration(60.0, 20.0 * M_PI);
/* Set up polling for every 100 milliseconds. */
     demoModule=makeModule("Demo", NULL);
     addPolling(demoModule, demoPoll, 100);
     fprintf(stderr, "done.\n");
}
/*===========================================*/
void
/* set up the shutdown function. */
commShutdown( )
{
     printf("somebody died. Exiting.\n");
     RaiShutdown( );
}
/*===========================================*/
/* This program starts up the robot, goes to */
/* a target point and quits after 30 seconds. */
int
main (int argc, char** argv)
{
     fprintf(stderr, "starting main\n");
     registerBaseClient( );
     colliRegister( );
     /* The shutdown function will be called */
     /* if baseServer dies. */
     initClient("colliTest", commShutdown);
     /* Hook up to running baseServer. */
     findBaseServer();
     colliConnect(1);
     /* Initialize, but do not start,*/
     /* the BeeSoft Scheduler. */
     RaiInit();
     catchInterrupts();
     /* Set up modules to do your communication. */
     initClientModules();
     /* Set up your user client application */
     /* to move the robot. */
     createDemoModule();
     /* Fire it up! */
     RaiStart();
     /* This will return only when RaiShutdown */
     /* is called, when <Ctrl/c> is pressed, */
     /* or when 30 seconds expire. */
     return;
}
/*===========================================*/ 
