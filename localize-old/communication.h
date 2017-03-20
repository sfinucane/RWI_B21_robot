
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/communication.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: communication.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.15  1999/05/21 14:48:46  fox
 * Added SEND_REPORT keyword.
 *
 * Revision 1.14  1999/05/18 15:15:19  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.13  1999/03/08 16:47:40  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.12  1998/08/26 15:34:03  wolfram
 * Finished integration of vision
 *
 * Revision 1.11  1998/08/20 00:22:56  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.10  1998/08/19 16:33:54  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.9  1998/04/08 16:36:00  wolfram
 * Changed the concept of multi-robot support. TCX now contains a procedure to
 * generate module names.
 *
 * Revision 1.8  1998/04/06 19:44:11  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.7  1997/12/02 15:20:35  fox
 * Nothing remarkable.
 *
 * Revision 1.6  1997/11/21 15:36:03  fox
 * Modifications in graphic
 *
 * Revision 1.5  1997/09/26 17:02:08  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.4  1997/05/27 07:42:31  fox
 * Nothing special.
 *
 * Revision 1.3  1997/03/13 17:36:35  fox
 * Temporary version. Don't use!
 *
 * Revision 1.2  1996/12/02 10:32:02  wolfram
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


#ifndef COMMUNICATION_INCLUDE
#define COMMUNICATION_INCLUDE

#include "general.h"
#include "sensings.h"
#include "localTcx.h"
#include "script.h"


typedef struct {
  script* scr;
  int connectBase;
  float timeFactor;
  int useTcx;
  char *robotName;
  char *moduleName;
  int subscribeBaseReport;
  int subscribeProximityReport;
  int odometryCorrection;
  int subscribeCameraReport;
  int sendCorrectionParametersToMap;
  int sendReports;
  int sendCorrectionParametersToPlan;
} informationsFor_COMMUNICATION;

extern informationsFor_COMMUNICATION communicationInfo;

void
initialize_COMMUNICATION( char* fileName,
			  int argc,
			  char **argv,
			  actionInformation* actionInfo,
			  sensingActionMask* actionMask);

bool
updateSensorInformation( actionInformation* info,
			 rawSensings* actualSensings,
			 abstractSensorVector* abstractSensors);

#endif















