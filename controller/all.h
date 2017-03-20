
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/controller/all.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:53:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: all.h,v $
 * Revision 1.1  2002/09/14 16:53:36  rstone
 * *** empty log message ***
 *
 * Revision 1.16  1999/09/28 01:39:52  thrun
 * connects to LOCALIZE and receives correction parameeters from there.
 *
 * Revision 1.15  1997/10/07 19:04:58  swa
 * DON'T make CD_VERSION the default
 *
 * Revision 1.14  1997/08/22 02:39:20  swa
 * Support for another CD (#ifdef UNIBONN is the old one).
 *
 * Revision 1.13  1997/08/16 23:10:42  thrun
 * Tourguide with CD
 *
 * Revision 1.12  1997/08/16 18:51:23  thrun
 * .
 *
 * Revision 1.11  1997/08/16 18:47:09  thrun
 * .
 *
 * Revision 1.10  1997/08/16 18:38:00  thrun
 * tourguide.
 *
 * Revision 1.9  1997/06/27 00:47:15  thrun
 * Excluded speech from the official BeeSoft release - this made it
 * necessary to change some of the files (those that included
 * speech stuff)
 *
 * Revision 1.8  1997/06/16 22:33:24  thrun
 * Now with cameraServer.
 *
 * Revision 1.7  1997/05/28 12:58:01  thrun
 * nicer colors
 *
 * Revision 1.6  1997/03/25 21:44:43  tyson
 * Many bug fixes.
 *
 * Revision 1.5  1997/02/02 22:32:31  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.4  1997/01/26 20:59:02  thrun
 * ARM is not part of the BeeSoft release any longer (compiles now
 * without ARM with the -DUNIBONN flag is not set).
 *
 * Revision 1.3  1996/11/27 23:20:20  thrun
 * (a) Modifications of Tyson's Makefile: they now work under Solaris again
 * (b) Major modifications of the CONTROLLER module.
 *
 * Revision 1.2  1996/09/23 09:51:24  thrun
 * Changes necessary to make the "-DUNIBONN" flag working again.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:27  rhino
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






/*#define TCX_debug*/

#ifdef UNIBONN
#undef UNIBONN
#endif


#define TOURGUIDE_VERSION	/* activate, if you'd like to configure
				 * the commander as a tourguide control unit.
				 * This is NOT a standard BeeSoft feature */

/* #define CD_VERSION */        /* activate, if you have a CD player installed
				 * on the robot.
				 * This is NOT a standard BeeSoft feature */


#include "tcx.h"
#include "tcxP.h"
#include "robot_specifications.h"
#include "imagesize.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"
#include "COLLI-messages.h"
#include "PANTILT-messages.h"
#include "MAP-messages.h"
#include "PLAN-messages.h"
#include "CAMERA-messages.h"
#include "SIMULATOR-messages.h"
#include "LOCALIZE-messages.h"
#ifdef UNIBONN
#include "ARM-messages.h"
#include "SUNVIS-messages.h"
#include "EZR-messages.h"
#include "EZR-defines.h"
#include "FLOW-messages.h"
#include "SPEECH-messages.h"
#endif /* UNIBONN */
#ifdef CD_VERSION
#include "struct.h"
#include "CD-tracks.h"
#include "CD-messages.h"
#endif /* CD_VERSION */

#include "BUTTONS-messages.h"


#include <bUtils.h>
#include "main.h"
#include "o-graphics.h"
#include "graphics.h"
#include "mouse.h"
#include "init.h"
#include "action.h"
#include "corr.h"
#ifdef UNIBONN
#include "task.h"
#endif /* UNIBONN */
#include "file.h"
#include "tour.h"



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)/2.0)
#define pi  3.14159265358979323846
#define pi2 6.28318530717958647692
