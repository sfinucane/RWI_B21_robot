
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/d1.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:48:15 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: d1.c,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1996/12/03 05:38:34  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:01  rhino
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


/******************************************************************************
*
* PROJECT: Carnegie Mellon Erebus Project
*          Task Control Architecture (TCX)
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
* 
* MODULE: d1
*
* FILE: d1.c
*
* ABSTRACT:
* 
* Try porblems with modules crashing d <-> b <-> a
*
* REVISION HISTORY:
*
* $Log: d1.c,v $
* Revision 1.1  2002/09/14 15:48:15  rstone
* *** empty log message ***
*
* Revision 1.2  1996/12/03 05:38:34  thrun
* If TCXHOST is not se, the software will now assume "localhost" and
* won't terminate.
*
* Revision 1.1.1.1  1996/09/22 16:46:01  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.1.1.1  1994/03/31 08:35:04  rhino
*
 * Revision 1.1.1.1  1994/03/17  14:22:28  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:30  rhino
 * test
 *
 * Revision 1.2  1993/03/12  20:58:19  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.1  1992/10/10  03:04:28  fedor
 * Moving toward basing initializing connections on moduleInfo messages.
 *
*
* NOTES:
*  none
*
******************************************************************************/

#include "stdio.h" 
#include "tcx.h"

#include "sampleTest.h"

extern char *getenv(char *);

void charMsgHnd(ref, sampleChar)
TCX_REF_PTR ref;
char *sampleChar;
{
  printf("sampleCharHnd: Start.\n");

  printf("ins: %d\n", tcxRefIns(ref));

  printf("sampleChar: %c\n", *sampleChar);

  printf("sampleCharHnd: End.\n\n");
}

/*************/

TCX_REG_HND_TYPE handlersArray[] = {
  {"charMsg", "charMsgHnd", charMsgHnd, TCX_RECV_ALL, NULL},
};

main()
{
  char *tcxMachine = NULL;
  
  TCX_MODULE_PTR module;

  printf("Connect ...\n");

  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize("d1", tcxMachine);

  /* module = tcxConnectModule("b1");*/

  tcxRegisterMessages(messageArray, 
		      sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(handlersArray,
		      sizeof(handlersArray)/sizeof(TCX_REG_HND_TYPE));
  
  tcxRecvLoop(NULL);
}

