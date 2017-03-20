
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/baseServer/sonarClient.h,v $
 *****
 ***** Created by:      $Author: tyson $
 *****
 ***** Revision #:      $Revision: 1.3 $
 *****
 ***** Date of revision $Date: 1997/07/17 17:31:42 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: sonarClient.h,v $
 * Revision 1.3  1997/07/17 17:31:42  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.2  1997/03/25 21:44:38  tyson
 * Many bug fixes.
 *
 * Revision 1.1  1996/11/18 04:34:56  tyson
 * More baseServer<->simulator2 work. Still not done.
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
 * Portions of this code were created by and are
 * copyrighted by Real World Interface Inc. (RWI)
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 *
 *   ==Contact  support@rwii.com  for further information==
 */

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

#ifndef SONAR_H
#define SONAR_H

#ifdef __cplusplus
extern "C" {
#endif


#include <sys/time.h>
#include <bUtils.h>

#define BOGUS_THRESHOLD 0
#define NO_RETURN 0x7FFF

/*
 * These structs may be expanded to support more features
 */

typedef struct sonarType {
  int value;			/* the distance */
  int mostRecent;		/* was this one in most recent return? */
  struct timeval time;		/* time of capture.  not yet precise */
} sonarType;

typedef  int (*sonarCallbackType)     (sonarType *sonar);

extern sonarType   sonars[];

void sonarInit(void);
void sonarStart(void);
void sonarStop(void);
void registerSonarCallback(sonarCallbackType);


#ifdef __cplusplus
}
#endif

#endif /* SONAR_H */
