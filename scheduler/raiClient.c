
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/scheduler/raiClient.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:37:37 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: raiClient.c,v $
 * Revision 1.1  2002/09/14 15:37:37  rstone
 * *** empty log message ***
 *
 * Revision 1.8  1998/01/17 01:06:21  swa
 * registeredMessages and registeredHandlers were of fixed size and there was
 * NO CHECK if clients exceed that size!!!!!!!!!!!!!!!!!!!!!!!!!!!!! :-(
 * Fixed.
 *
 * Revision 1.7  1997/03/11 17:16:41  tyson
 * added IR simulation and other work
 *
 * Revision 1.6  1997/02/22 05:16:52  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.5  1997/02/22 00:59:10  thrun
 * Introduced version number support
 *
 * Revision 1.4  1997/02/12 13:11:22  tyson
 * more parameter utils.  Updated wander.c. Some processes now support -fork=[y|n] instead of [+|-]stdin
 *
 * Revision 1.3  1997/02/02 22:32:42  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.2  1996/12/12 02:17:59  tyson
 * simulator2<->baseServer work
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

 
#ifndef lint
static char rcsid[] = "$Id: raiClient.c,v 1.1 2002/09/14 15:37:37 rstone Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <tcx.h>
#include <rai.h>
#include <raiClient.h>
#include <bUtils.h>

#include "beeSoftVersion.h"


extern void tcxRegisterCloseHnd(void (*closeHnd)());

/* change this to linked list later */
#define X_LIST_SIZE 200
TCX_REG_MSG_TYPE  registeredMessages[X_LIST_SIZE];
TCX_REG_HND_TYPE  registeredHandlers[X_LIST_SIZE];
int nextMessage = 0;
int nextHandler = 0;


void
registerInterface(char* name, int numMsgs,TCX_REG_MSG_TYPE messages[],
		  int numHnds,TCX_REG_HND_TYPE handlers[])
{
  int i;

  for(i=0;i<numMsgs;i++) {
    if (nextMessage >= X_LIST_SIZE){
      fprintf(stderr, 
	      "Too many entries in raiClient.c. Change X_LIST_SIZE.\n");
      exit(-1);
    }      

    registeredMessages[nextMessage].msgName = strdup(messages[i].msgName);
    
    if (messages[i].msgFormat) {
      registeredMessages[nextMessage].msgFormat =
	strdup(messages[i].msgFormat);
    }
    else {
      registeredMessages[nextMessage].msgFormat = NULL;
    }
    nextMessage++;
  }
  
  for(i=0;i<numHnds;i++) {
    if (nextHandler >= X_LIST_SIZE){
      fprintf(stderr, 
	      "Too many entries in raiClient.c. Change X_LIST_SIZE.\n");
      exit(-1);
    }      
    registeredHandlers[nextHandler].msgName =	strdup(handlers[i].msgName);
    registeredHandlers[nextHandler].hndName = strdup(handlers[i].hndName);
    registeredHandlers[nextHandler].hndProc = handlers[i].hndProc;
    registeredHandlers[nextHandler].hndControl = handlers[i].hndControl;
    registeredHandlers[nextHandler].hndData = handlers[i].hndData;
    nextHandler++;
  }
/*   fprintf(stderr, "[[ %d handlers ]]\n", nextHandler); */
}

void closeClient()
{
 tcxCloseAll();
}


void
shutdownClient()
{
  tcxCloseAll();	
}


void
initClient(char* name,clientCloseFcn clientClose)
{

  if(nextMessage == 0) {
    fprintf(stderr, "RaiClient: initClient(): "
	    "No interface messages registered.\n");  
    return;
  }

  if (!bRobot.TCXHOST) {
    fprintf(stderr, "RaiClient: initClient(): "
	    "TCXHOST not specified.  Assuming 'localhost'.\n");
    tcxInitialize(name, "localhost");
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			 NULL, 1);
  

  }
  else {
    tcxInitialize(name,(char *)bRobot.TCXHOST);
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE, 
			 NULL, 1);
  }

  tcxRegisterMessages(registeredMessages,nextMessage);



  if(nextHandler >0) {
/*     int i; */
/*     for(i=0;i<nextHandler;i++) { */
/*       fprintf(stderr, "<%d: [%s] [%s] %p %p %p>\n", */
/* 	      i,  */
/* 	      registeredHandlers[i].msgName, */
/* 	      registeredHandlers[i].hndName, */
/* 	      registeredHandlers[i].hndProc, */
/* 	      registeredHandlers[i].hndControl, */
/* 	      registeredHandlers[i].hndData); */
/*     } */
/*     fprintf( stderr,"-f-\n"); */
    tcxRegisterHandlers(registeredHandlers,nextHandler);
  }

  tcxRegisterCloseHnd(clientClose);
}

void 
clientTCXCallback(RaiModule * mod)
{
  struct timeval TCX_waiting_time = {0, 10};
  tcxRecvLoop((void *) &TCX_waiting_time); 
}
 
void
initClientModules()
{
  RaiModule *client_module;
  client_module = makeModule("ClientTCXModule",NULL);
  addPolling(client_module,clientTCXCallback,TCX_CLIENT_POLLING_INTERVAL);
}
