
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection/graphics.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:44:56 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: graphics.h,v $
 * Revision 1.1  2002/09/14 20:44:56  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1998/08/29 21:44:43  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.3  1998/08/23 22:57:40  fox
 * First version of building maps of humans.
 *
 * Revision 1.2  1997/05/06 14:22:58  fox
 * Nothing special.
 *
 * Revision 1.1  1997/05/05 16:54:07  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.20  1997/03/13 17:36:35  fox
 * Temporary version. Don't use!
 *
 * Revision 1.19  1997/02/13 18:30:13  thrun
 * Dieter and Wolfram: Do you know htat MAXFLOAT is not defined under SUN
 * OS 4.1.3?
 *
 * Revision 1.18  1997/02/12 15:08:36  fox
 * Integrated laser support.
 *
 * Revision 1.17  1997/02/11 11:04:08  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.16  1997/01/31 16:19:16  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.15  1997/01/29 12:23:07  fox
 * First version of restructured DETECTION.
 *
 * Revision 1.14  1997/01/18 18:19:22  wolfram
 * *** empty log message ***
 *
 * Revision 1.13  1997/01/07 08:52:08  wolfram
 * Added time to movements and struct realCellList
 *
 * Revision 1.12  1997/01/06 16:30:47  wolfram
 * Added time stamp to proximity sensings
 *
 * Revision 1.11  1997/01/03 10:09:46  fox
 * First version with exploration.
 *
 * Revision 1.10  1996/12/19 14:33:28  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.9  1996/12/02 10:32:05  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.8  1996/11/28 17:56:22  fox
 * *** empty log message ***
 *
 * Revision 1.7  1996/11/26 11:08:11  fox
 * Improved version.
 *
 * Revision 1.6  1996/11/21 16:03:54  wolfram
 * Added extern c for c++
 *
 * Revision 1.5  1996/11/21 14:55:49  wolfram
 * added gridCellList to general.h
 *
 * Revision 1.4  1996/11/21 12:40:25  fox
 * Tools for bayesian reasoning on the grids.
 *
 * Revision 1.3  1996/11/18 09:58:29  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/10/24 12:07:10  fox
 * Fixed a bug.
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


#ifndef GRAPHICS_INCLUDE
#define GRAPHICS_INCLUDE

#include "general.h"

extern int LOCAL_DETECTION;
extern int PERSON_BUTTON;
extern int MODE_BUTTON;
extern int QUIT_BUTTON;
extern int BASE_CONNECTED_BUTTON;
extern int LOCALIZE_CONNECTED_BUTTON;
extern int SOUND_CONNECTED_BUTTON;
extern int MOUTH_CONNECTED_BUTTON;
extern int PANTILT_CONNECTED_BUTTON;
extern int PERSON;


typedef struct {
  EZXW_p window;
  int sizeX;
  int sizeY;
  int winSizeX;
  int winSizeY;
  int scale;
}  gridWindow;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void 
initGraphics(ROBOT_STATE_PTR    robot_state,
	       PROGRAM_STATE_PTR  program_state,
	       ROBOT_SPECIFICATIONS_PTR robot_specifications);

int 
mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		PROGRAM_STATE_PTR        program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications);

void
displayDetection( detectionStruct* obstacles);

void
displayHumanProbFunction( humanProbTable humanProbs);

gridWindow*
createMapWindow( char* text, 
		 int x, int y, int scale);

void
displayMapWindow( probabilityGrid *m, gridWindow *mapWin, 
		  realPosition* rpos);

#endif

