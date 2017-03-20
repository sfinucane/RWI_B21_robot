
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/baseServer/statusReport.c,v $
 *****
 ***** Created by:      $Author: tyson $
 *****
 ***** Revision #:      $Revision: 1.8 $
 *****
 ***** Date of revision $Date: 1997/07/25 20:54:49 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: statusReport.c,v $
 * Revision 1.8  1997/07/25 20:54:49  tyson
 * new bCheck and bWatch.  Fixed units problems in simulator and baseServer
 *
 * Revision 1.7  1997/07/17 17:31:42  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.6  1997/03/25 21:44:38  tyson
 * Many bug fixes.
 *
 * Revision 1.5  1997/03/11 17:03:18  tyson
 * added IR simulation and other work
 *
 * Revision 1.4  1997/02/25 18:12:37  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.3  1997/01/28 20:39:36  tyson
 * daemonize COLLI, other tweaks
 *
 * Revision 1.2  1996/12/06 05:55:00  tyson
 * simulator2 <-> baseServer mostly working for motion and sonar
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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>

#include <base.h>
#include <rai.h>  /* raigettime, raisettime */ 
#include <bUtils.h>

#define MAX_CALLBACKS 20
typedef void (*func_ptr_type)(statusReportType*);
typedef void (*wd_func_ptr_type)(void);

void* callbacks[MAX_CALLBACKS];
statusReportType activeStatusReport;
statusReportType * report;

#undef DEBUG



void initStatusStructure()
{
  int i;
  report  = &activeStatusReport;
  memset(report, 0, sizeof(*report));
  activeStatusReport.BaseRelativeHeading = 2048;
  setRaiTime();
  for (i = 0; i < MAX_CALLBACKS; i++)
    callbacks[i] = NULL;
}


void regStatusCallback(void* function_ptr)
{
  int i = 0;

  while ((callbacks[i] != 0)  && (i<MAX_CALLBACKS))
    i++;

  if (i < MAX_CALLBACKS)
    callbacks[i] = function_ptr;
}


void handleStatusCallbacks()
{
  void (*callback_function)(statusReportType*);
  int i;
  for (i = 0; i < MAX_CALLBACKS; i++)
    {
      if ((callbacks[i]) != 0)
	{
	  callback_function = (func_ptr_type)(callbacks[i]);
	  (*callback_function)(report);
	}
    }
}



/* copies 2 bytes in net order into a long in host order */
/* and returns the pointer to the next unused byte */

char* copy2Bytes(unsigned long * dest, char* src)
{
      *dest = 0;
      memcpy((char*)(dest)+2,src,2);
      *dest = htonl(*dest);
      return src+2;
}

statusReportType* parseReport(unsigned char* status_values)
{
  unsigned long time;
  unsigned long itemsReported;
  unsigned char* string_position = (status_values + 1);

  time = getRaiTime();

#ifdef DEBUG
  fprintf(stderr,"entering parseReport\n");
#endif

  /* first long is bit flag for rest of items */
  memcpy(&(itemsReported),string_position,4);
  string_position += 4;

  itemsReported = htonl(itemsReported);
  report->Request = itemsReported;

#ifdef DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): SRD = 0x%04X\n",
	  __FILE__, __LINE__, __FUNCTION__, itemsReported);
#endif    

  /**** these must remain in numerical order !! *******/
  /**** as the base will pass them back that way ******/

  if (itemsReported & REPORT_BASE_CLOCK) {
    memcpy(&(report->Clock),string_position,4);
    report->Clock = htonl(report->Clock);

#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): Clock = 0x%04X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->Clock);
#endif    
    string_position += 4;
  }

  if (itemsReported & REPORT_GENERAL_STATUS) {
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): ****\n",
	    __FILE__, __LINE__, __FUNCTION__);
#endif    
    string_position  = copy2Bytes(&report->GeneralStatus,string_position);
  }

  if (itemsReported & REPORT_X) {
    /* NOTE:  Ramona (Browns Base) is backwards, so X coordinated must */
    /*        corrected.  It is a 32 bit unsigned long, so negating it */
    /*        will cause X to go in the other direction */
    
    string_position  = copy2Bytes(&report->Xpos,string_position);

    if (bRobot.base_rotBackwards) {
      report->Xpos = -report->Xpos;
    }

#ifdef DEBUG    
    fprintf(stderr, "%10s:%5d:%14s(): base-X = 0x%08X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->Xpos);
#endif    

    /* This ugliness is caused by having to make sure that we don't get any */
    /* arithmetic silliness such as casting bRobot.base_posPerCm before we  */
    /* actually use it.  Yuck!                                              */
    report->Xpos = (unsigned long)((float)(report->Xpos) / 
                                   bRobot.base_posPerCm * 10.0);
#ifdef DEBUG    
    fprintf(stderr, "%10s:%5d:%14s(): base-X = 0x%08X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->Xpos);
#endif

  }

  if (itemsReported & REPORT_Y) {
    string_position  = copy2Bytes(&report->Ypos,string_position);
#ifdef DEBUG    
    fprintf(stderr, "%10s:%5d:%14s(): base-Y = %4X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->Ypos);
#endif    
    /* Same ugliness as above */
    report->Ypos = (unsigned long)((float)(report->Ypos) / 
                                   bRobot.base_posPerCm * 10.0);
#ifdef DEBUG
    fprintf(stderr,"Status Yposition=%d",report->Ypos); 
#endif
  }

  if (itemsReported & REPORT_HEADING) {
    string_position  = copy2Bytes(&report->Heading,string_position);
#ifdef DEBUG    
    fprintf(stderr, "%10s:%5d:%14s(): base-Heading = %4X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->Heading);
#endif    

    if (bRobot.base_rotBackwards) {
      report->Heading = 1024 -  report->Heading;
    }
  }

  if (itemsReported & REPORT_BASE_RELATIVE_HEADING) {
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): ****\n",
	    __FILE__, __LINE__, __FUNCTION__);
#endif    
    string_position  = copy2Bytes(&report->BaseRelativeHeading,
				  string_position);

    if (bRobot.base_rotBackwards) {
      report->BaseRelativeHeading = 1024 -  report->BaseRelativeHeading;
    }
  }

  if (itemsReported & REPORT_TRANSLATE_ERROR) {
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): ****\n",
	    __FILE__, __LINE__, __FUNCTION__);
#endif    
    string_position  = copy2Bytes(&report->TranslateError,string_position);
    report->TranslateError /= (bRobot.base_encPerCm/10.0);
    }
  
  if (itemsReported & REPORT_TRANSLATE_VELOCITY) {
    string_position=copy2Bytes(&report->TranslateVelocity,string_position);
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): TVE = 0x%04X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->TranslateVelocity);
#endif    
    report->TranslateVelocity /= (bRobot.base_encPerCm/10.0);
  }
  
  if (itemsReported & REPORT_TRANSLATE_STATUS) {
    string_position=copy2Bytes(&report->TranslateStatus,string_position);
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): TSF = 0x%04X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->TranslateStatus);
#endif    
  }
  
  if (itemsReported & REPORT_ROTATE_ERROR) {
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): ****\n",
	    __FILE__, __LINE__, __FUNCTION__);
#endif    
    string_position  = copy2Bytes(&report->RotateError,string_position);
  }

  if (itemsReported & REPORT_ROTATE_VELOCITY) {

    string_position  = copy2Bytes(&report->RotateVelocity,string_position);
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): RVE = 0x%04X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->RotateVelocity);
#endif    
  }

  if (itemsReported & REPORT_ROTATE_STATUS) {
    string_position=copy2Bytes(&report->RotateStatus,string_position);
#ifdef DEBUG
    fprintf(stderr, "%10s:%5d:%14s(): RSF= 0x%04X\n",
	    __FILE__, __LINE__, __FUNCTION__, report->RotateStatus);
#endif
  }


  handleStatusCallbacks();

#ifdef DEBUG
  fprintf(stderr,"exiting parse report \n");
#endif

  return report;
}
