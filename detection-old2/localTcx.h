
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old2/localTcx.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: localTcx.h,v $
 * Revision 1.1  2002/09/14 20:45:04  rstone
 * *** empty log message ***
 *
 * Revision 1.9  1998/09/05 00:25:28  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.8  1998/08/29 21:44:44  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.7  1998/08/23 22:57:41  fox
 * First version of building maps of humans.
 *
 * Revision 1.6  1997/06/03 11:49:15  fox
 * Museum version.
 *
 * Revision 1.5  1997/05/25 10:40:54  fox
 * Nothing special.
 *
 * Revision 1.4  1997/05/09 16:28:40  fox
 * Works quiet fine.
 *
 * Revision 1.3  1997/05/06 14:22:59  fox
 * Nothing special.
 *
 * Revision 1.2  1997/05/05 16:54:08  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.10  1997/03/13 17:36:37  fox
 * Temporary version. Don't use!
 *
 * Revision 1.9  1997/01/31 17:11:02  fox
 * Integrated laser reply.
 *
 * Revision 1.8  1997/01/30 13:34:12  fox
 * Minor changes.
 *
 * Revision 1.7  1997/01/29 12:23:09  fox
 * First version of restructured DETECTION.
 *
 * Revision 1.6  1997/01/06 17:38:41  fox
 * Improved version.
 *
 * Revision 1.5  1996/12/31 09:19:24  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.4  1996/12/20 15:29:39  fox
 * Added four parameters.
 *
 * Revision 1.3  1996/12/13 13:55:37  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.2  1996/12/02 10:32:08  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#ifndef LOCALTCX_INCLUDE
#define LOCALTCX_INCLUDE

#include "tcx.h"
#include "tcxP.h"
#include "detection.h"
#include "localize.h"

extern bool connectBase;
extern bool connectPantilt;
extern bool connectSound;
extern bool connectMouth;
extern bool tcx_initialized;
extern bool mapPositionKnown;
extern bool laserPositionKnown;

extern realPosition robotPosition;
extern float uncorrectedRobotRot;
extern bool robotInMotion;
extern bool targetPointGiven;

extern float correctionX, correctionY, correctionRot;
extern int correctionType;

extern float rotationalSpeed;

void
init_tcx();

bool
getSensing();

void 
broadcastStatusReport( detectionStruct detection);

void
triggerSound();

void
sayStuck();

#define NUMBER_OF_MOODS 4

void
setMood( int mood);

#define BASE_DEFAULT_MODE 0
#define BASE_ESCAPE_MODE 9

void
setBaseMode( int mode);

#endif


