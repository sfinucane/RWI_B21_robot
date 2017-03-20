
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/localTcx.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: localTcx.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.24  1999/11/02 18:12:35  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.23  1999/10/21 17:30:44  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.22  1999/09/09 02:48:38  fox
 * Final version before germany.
 *
 * Revision 1.21  1999/09/06 16:36:03  fox
 * Many changes.
 *
 * Revision 1.20  1999/09/03 22:22:39  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.19  1999/04/28 22:15:59  fox
 * The script can now contain robot detection information. If such a detection
 * is found, then it is sent to the module MULTI_LOCALIZE (not perfect though).
 *
 * Revision 1.18  1998/10/29 03:45:03  fox
 * Nothing special.
 *
 * Revision 1.17  1998/08/31 22:29:21  wolfram
 * Several changes
 *
 * Revision 1.16  1998/08/20 00:23:00  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.15  1998/08/19 16:33:56  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.14  1998/04/08 16:36:02  wolfram
 * Changed the concept of multi-robot support. TCX now contains a procedure to
 * generate module names.
 *
 * Revision 1.13  1998/04/06 19:44:13  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.12  1997/05/26 09:34:15  fox
 * Replaced entropy by information.
 *
 * Revision 1.11  1997/05/09 16:28:18  fox
 * Nothing special.
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
 * First version of restructured LOCALIZE.
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
#include "sensings.h"
#include "localize.h"


#define LENGTH_OF_MODULE_NAMES 128
#define NUMBER_OF_TCX_MODULES 7
#define USER_MODULE 0
#define BASE_MODULE 1
#define PLAN_MODULE 2
#define BUTTONS_MODULE 3
#define CAMERA_MODULE 4
#define SOUND_MODULE 5
#define MAP_MODULE 6 

extern char moduleName[NUMBER_OF_TCX_MODULES][LENGTH_OF_MODULE_NAMES];


extern bool tcx_initialized;


void
initializeModuleNames(char *robotName);

void
init_tcx( actionInformation* info,
	  bool connectBase,
	  int subscribeBaseReport,
	  int subscribeProximityReport,
	  int subscribeCamera);

#define DONT_WAIT 0.0

void
swallowStatusReports( float secs);

bool
getSensing( rawSensings* actualSensing);

#define CONSIDER_PLAN      1
#define DONT_CONSIDER_PLAN 0
void
broadcastStatusReport( sensingActionMask* mask,
		       bool considerPlan);

void
sendBasePositionToUpdateModules( realPosition robPos);

void
sendLaserToUpdateModules( sensing_PROXIMITY frontLaser,
			  sensing_PROXIMITY rearLaser);

bool
sendCorrectionParams( correctionParameter corr,
		      TCX_MODULE_PTR module,
		      char* name);

bool
sendMap( probabilityGrid* map,
	 bool inverted,
	 TCX_MODULE_PTR module,
	 char* name);


bool
sendMapReplyType( probabilityGrid* map,
		  bool inverted,
		  TCX_MODULE_PTR module,
		  char* name, int type);


void
setModuleName( char *name, char *robot, char *module);



/* This function checks wether the connection with the module is established
 * yet. If not it will try to start the connection. */
bool
connectionEstablished( TCX_MODULE_PTR* module, char* name);

#define MAX_NUMBER_OF_TCX_SAMPLES 5000

void
sendDetectionToMultiLocalize( char* observer, char* detected,
			      float dist, float distUncertainty,
			      float angle, float angleUncertainty,
			      float currentRotSpeed,
			      struct timeval timeStamp,
			      int detectionFlag);


#endif






