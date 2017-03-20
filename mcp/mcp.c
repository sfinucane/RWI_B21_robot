
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/mcp/mcp.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:29:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: mcp.c,v $
 * Revision 1.1  2002/09/14 15:29:36  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1997/03/28 03:48:31  tyson
 * finding .ini files and minor stuff
 *
 * Revision 1.3  1997/03/11 17:14:11  tyson
 * added IR simulation and other work
 *
 * Revision 1.2  1997/01/24 21:42:28  tyson
 * fixed serious libmcp bug, added bUtils for parameter files, logging and daemonizing plus misc.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:09  rhino
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



/****************************************************************/
/*    ACCT: jmk                                                 */
/*    FILE: Mcp.c                                               */
/*    DATE: April 1995                                          */
/*                                                              */
/*    Modification history:                                     */
/*                                                              */
/*  8/2/95 jmk  Took out sending 6 zeros to MCP on every command*/
/*              This is only necessary on error.  Also, put     */
/*              carriage return before ASCII command to go into */
/*              binary mode.  This helps if MCP is in ascii mode*/
/*              and has junk on its input line                  */
/*                                                              */
/*  7/5/95  jmk Added baudRate arg to openMcp.  The MCP connect */
/*              will be at the specified baud rate.  Note that  */
/*              the baud rate must be specified by one of the   */
/*              flags defined in termios.h, like B9600 for 9600 */    
/*                                                              */
/*  6/24/95 jmk Added alarm syscall around the code that sends  */
/*              the first command to the mcp.  Now, if the mcp  */
/*              is off, in need of a reset, etc, the alarm      */
/*              expires while we are trying to contact it. So,  */
/*              we give a warning to reset the base, rather than*/
/*              hanging on its repsonse.                        */ 
/*                                                              */
/* 4/95    jmk  Took mcp  specific code out of base module and  */
/*              made a separate library to be used by any mcp   */
/*              related module (base, arm etc)                  */
/****************************************************************/

#include "mcpIO.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>    /* for byte-order */
#include <signal.h>
#undef DEBUG
#undef VERBOSE


#define START 2
#define END 3


/* STATE OF MCP IO PACKET */

#define NO_STARTS 1
#define ONE_START 2
#define IN_PROGRESS 3
#define ONE_END 4

/* We set an alarm for this many seconds before attempting to contact */
/* the mcp.  If the mcp is off, dead, etc the alarm will expire and */
/* we can bag out.  If not, we reset the alarm and continue */ 

#define MCP_WAIT_TIME 5


void setBinaryMode(mcpIOState* state)
{
  char* message = "\rBE FFFF\rDM\r";
  int file_success = 0;
  unsigned char return_text[20]; 
  int file_id = state->fd;

  file_success = write(file_id, message, strlen(message));

  /* sleep for a quarter second to let any ASCII commands the MCP is */
  /* processing to end */
  usleep(250000);

  /* clear out any crap that might have been left over from ASCII mode */ 
  for(;;)
    {
      file_success = read(file_id, return_text, 1);
      if (file_success == -1)
	{
	  fprintf(stderr,"Error reading from MCP\n");
	  exit(-1);
	}
      if (!file_success) break;
    }
}


void deadMcpHandler()
{
  fprintf(stderr,
	  "\a\aERROR: Motor controller did not respond in %d seconds.\n",
	  MCP_WAIT_TIME);

  fprintf(stderr,
	  "ERROR: Perhaps the base/arm needs to be reset.\n");
  exit(-1);
}


mcpIOState* openMcp(const char* device, char* lockname,speed_t baudRate)
{
  int file_id;
  mcpIOState* state;

  if (makeLock(lockname)<0) 
    {
      fprintf(stderr,"openMcp: Failed to get lock %s\n",lockname);
      return NULL;
    }
  
  file_id = openRaw(device, O_RDWR);
  if (file_id < 0)
    {
      fprintf(stderr,"openMcp: Failed to open %s\n",device);
      return NULL;
    }

  /* In case base is hosed. SIGALRM will be thrown. If we catch it,*/
  /* MCP did not respond in time.  If not, cancel alarm */
  signal(SIGALRM, &deadMcpHandler); 
  alarm(MCP_WAIT_TIME);

  /* Set the input and output baud rates to that which was
     specified on entry to openMcp(). */
  if (!setBaudRate(file_id, baudRate))
     fprintf(stderr, "openMcp(): Unable to set the baud rate of "
             "%s!\n", (const char*)device);
  
  state = (mcpIOState*) calloc(1,sizeof(mcpIOState));
  state->fd = file_id;
  state->lockname = strdup(lockname);
  state->return_string = (char*) calloc(512,sizeof(char));
  state->state = NO_STARTS;
  state->start_in_stream = FALSE;
  state->pos = 0;
  state->buffLen = 0;
  state->buffPos = 0;

  /* will definitely choke if base is off */
  setBinaryMode(state);

  /* if we made it back, cancel alarm */
  alarm(0);
  signal(SIGALRM, SIG_DFL);
  resetMcpComm(state);
  return state;
}


void closeMcp(mcpIOState* state)
{
  releaseLock(state->lockname);
  close(state->fd);
}

void resetMcpComm(mcpIOState* state)
{
  int file_id = state->fd;
  char blanks[8] = { 0,0,0,0,0,0,0,0 };
  write(file_id, blanks, 7);
}

void mcpSendCmd(mcpIOState* mcp,unsigned char opCode, unsigned long parameter)
{
  unsigned char message[7];
  unsigned long fixedParam = 0;
  int file_id = mcp->fd;

  message[0]=opCode; 
  
  if (parameter)
    fixedParam= htonl(parameter);

  memcpy(message+1,&fixedParam, 4);

  /* 5th byte is a checksum */
  message[5] = message[0] ^ message[1] ^ message[2] ^ message[3] ^ message[4];

  write(file_id, message, 6);
}


char* mcpSelect(mcpIOState* state)
{
  char* message = NULL;
  int bytes = 0;
  char thisChar;

  if (state->buffLen == 0) {
    bytes = read(state->fd,state->buff,511);

    if (bytes == -1) {
      fprintf(stderr,"Error on MCP file descriptor: BRACE FOR IMPACT :)\n");
      return NULL;
    }

    if (bytes>0) {
      state->buffLen = bytes;
      state->buffPos = 0;
    }
#ifdef DEBUG
    fprintf(stderr,"Read %d bytes from mcp\n",bytes);
#endif
  }

  while ((state->buffPos < state->buffLen) && (!message)) {
    thisChar = state->buff[state->buffPos++];
#ifdef DEBUG
    fprintf(stderr,"Working char %d (%ud)\n",i,0xff & thisChar);
#endif
    
    switch(state->state) {
      
    case NO_STARTS:
      
      if (thisChar==START) {
	state->state = ONE_START;
	state->return_string[state->pos]=thisChar;
	state->pos++;
      }
      else {
#ifdef VERBOSE
	fprintf(stderr,"\aUnexpected char %x before start\n",thisChar);
#endif
      }
      break;
      /************************/
    case ONE_START:
      if (thisChar==START) {
	state->return_string[state->pos]=thisChar;
	state->pos++;
	state->state = IN_PROGRESS;
      }
      else {
	state->state = NO_STARTS; /* starts only count if in a pair */
#ifdef VERBOSE
	fprintf(stderr,"\aUnexpected char %x before start\n",thisChar);
#endif
      }
      
      break;
      /************************/
      
    case IN_PROGRESS:
      
      state->return_string[state->pos]=thisChar;	
      state->pos++;
      
      switch(thisChar) {
      case END:
	state->state = ONE_END;
	break;
	
      case START:
	if (state->start_in_stream) {
	  /* somebody screwed the pooch, because we just got 2 starts*/
	  /* maybe we missed the last end */
#ifdef VERBOSE
	  fprintf(stderr,"\aUnexpected start in base packet\n");
#endif
	  state->start_in_stream = FALSE;
	  state->state = NO_STARTS;
	  state->pos=0;
	}
	else {
	  state->start_in_stream = TRUE;
	  break;
	}
	
      default:
	state->start_in_stream = FALSE;
	break;
      }
      
      break;
      /************************/
      
    case ONE_END:
      state->return_string[state->pos]=thisChar;
      state->pos++;
      
      if (thisChar==END) {
	state->return_string[state->pos]=0;
	state->state = NO_STARTS;
	state->pos=0;
	message = state->return_string;
	state->return_string = (char*) calloc(512,sizeof(char));	    
      }
      else {
	if (thisChar==START)
	  state->start_in_stream = TRUE;
	state->state = IN_PROGRESS;
      }
    }
  }
  
  if (state->buffPos >= state->buffLen) {
    state->buffPos = 0;
    state->buffLen = 0;
  }

#ifdef DEBUG
  fprintf(stderr,"Done with select\n");
#endif

#if 0
  fprintf(stderr, "%s:%s(%d) - returning(message) = 0x%08X\n",
	  __FILE__, __FUNCTION__,  __LINE__, message);
#endif

  return message;
}


