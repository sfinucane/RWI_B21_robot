
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/utils/orig/battery/checkBattery.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:51:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: checkBattery.c,v $
 * Revision 1.1  2002/09/14 15:51:39  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:21  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

#include <syslog.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <rai/Rai.h>
#include <rai/Base.h>
#include <rai/baseMessages.h>

/* Base.h contains an explanation of the voltage constants WARNING_VOLTAGE */
/* PANIC_VOLTAGE, MIN_BELIEVABLE_ and MAX_BELIEVABLE_VOLTAGE */

/* gethostname, used below, takes a buffer and puts the name of the system */
/* in it.  This is the length of the buffer passed */
#define MAX_HOST_NAME_LEN 512   

unsigned long avgBatteryVoltage = 0;
unsigned long lastBatteryVoltage = 0;
unsigned long silent = FALSE;   /* suppress printing voltage to stdout */

void batteryCallbackFcn (unsigned long OpCode, unsigned long Value)
{
  if (OpCode == BASE_batteryVoltage)
    lastBatteryVoltage=Value;
}

void battery_poll(RaiModule * mod) 
{
  static int goodValues = 0;
  
  /* the first time we call this lastBatteryVoltage will be 0 */
  /* since we may not have asked for any battery voltages yet */ 

  if (lastBatteryVoltage &&
      (lastBatteryVoltage <= MAX_BELIEVABLE_VOLTAGE) &&
      (lastBatteryVoltage >= MIN_BELIEVABLE_VOLTAGE))
    {
      avgBatteryVoltage = 	
	((avgBatteryVoltage * goodValues) + lastBatteryVoltage)/(goodValues+1);
      goodValues +=1;
    }
  /* ask to get next battery voltage, which will call batteryCallbackFcn. */
  /* we aren't guaranteed that the two will remain in synch, but it is   */
  /* close enough */
  batteryVoltage();
}


void battery_timeout(RaiModule * mod)
{
  RaiShutdown();
}


void init_battery()
{
   RaiModule * battery_module;
   battery_module = makeModule("battery",NULL);
   addPolling(battery_module,battery_poll,800);
   addTimeout(battery_module,battery_timeout,6000);
}




void forkRemoteShutdown(char* computer)
{

  int pid;

  pid = fork();  /* create a second process */

  if (pid ==0)  /* Child process calls this.  Execlp will not return */
  {
    if (strcmp(computer, "localhost")) {
      execlp("/usr/bin/rsh","rsh", computer, "-n", "-K", 
	     "/usr/local/bin/poweroff", NULL);
    }
    else {
      execlp("/usr/local/bin/poweroff", "poweroff", NULL);
    }
      
    /* should never call this, unless the above failed */
    syslog(LOG_EMERG,"Voltage Daemon: remote shutdown failed");
    exit(-1);  /* kill child process */
  }
}



void  findVoltage()
{
  char * RightComputer;
  char * LeftComputer;
  char * LaptopComputer;
  char * BaseComputer;
  FILE * localLogFile;
  char  ThisComputer[MAX_HOST_NAME_LEN];
  int error = 0;


  localLogFile = fopen("lastVoltage.txt","w");

  error=gethostname(ThisComputer,MAX_HOST_NAME_LEN);
  if (error != 0)
    {
      syslog(LOG_WARNING,
	     "Voltage Daemon: gethostname failed, errno = %d\n",errno);
      return;
    }

  BaseComputer =getenv("BASEHOST");    

  if ((BaseComputer == NULL) || strcmp(BaseComputer,ThisComputer))
    {
     syslog(LOG_WARNING,
	    "Voltage Daemon: BASE_ON not set or not running on base computer\n");
     return;
    }


  if (localLogFile != NULL)
    fprintf(localLogFile,"Starting RAI to check battery voltage\n");
  if (!silent)
    printf("Starting RAI to check battery voltage\n");

  RaiInit();
  BaseInit();

  init_battery();

  /* this will be called whenever the base wants to give us some data*/
  /* like the battery voltage info we ask for */
  registerBaseCallback(batteryCallbackFcn);
  
  RaiStart();   /* this will go until battery module calls RaiShutdown() */
  

  /* If we get here, RaiShutdown was called and voltage value was computed */

  if (!silent)
    {
      printf("Battery voltage\t%lu\n", avgBatteryVoltage);
      printf("Danger level is\t%d\n", WARNING_VOLTAGE);
    }

  if (localLogFile != NULL)
    fprintf(localLogFile,"Battery voltage=%lu\n", avgBatteryVoltage);

  if (localLogFile != NULL)
    fclose(localLogFile);

  if ((avgBatteryVoltage >= MAX_BELIEVABLE_VOLTAGE) ||
      (avgBatteryVoltage <= MIN_BELIEVABLE_VOLTAGE))
    return;


  /* some of these may be NULL as we could have less than 3 computers */
  LaptopComputer =getenv("CONSOLE_COMPUTER");
  LeftComputer   =getenv("LEFT_COMPUTER");
  RightComputer  =getenv("RIGHT_COMPUTER");

  if (avgBatteryVoltage <= PANIC_VOLTAGE)
    {
      syslog(LOG_EMERG,"Voltage Daemon: Base voltage is %lu. Shutting down\n",
	     avgBatteryVoltage);
      
      if ((LaptopComputer != NULL) && strcmp(LaptopComputer,ThisComputer))
	forkRemoteShutdown(LaptopComputer);

      if ((RightComputer != NULL) && strcmp(RightComputer,ThisComputer))
	forkRemoteShutdown(RightComputer);

      if ((LeftComputer != NULL) && strcmp(LeftComputer,ThisComputer))
	forkRemoteShutdown(LeftComputer);

      sleep(60);  /* give above shutdowns chance to work */

      forkRemoteShutdown(ThisComputer);  
      sleep(1024);  /* give our own shutdown a chance to work */

      syslog(LOG_EMERG,"Voltage Daemon: Local Shutdown Failed!!!");
      return;
    }
  else if (avgBatteryVoltage<=WARNING_VOLTAGE)
    {
      syslog(LOG_WARNING,"Voltage Daemon:  Base voltage has dropped to %lu.",
	     avgBatteryVoltage);
      syslog(LOG_WARNING,"Voltage Daemon:  Shutdown will occur at %lu volts.",
	     PANIC_VOLTAGE);
      system("echo Voltage Daemon detects dangerously low voltage | wall"); 
    }
}


void main(int argc, char** argv)
{
  if (argc>1) 
    silent = TRUE;
  findVoltage();
}











