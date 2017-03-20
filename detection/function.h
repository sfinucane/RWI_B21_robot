
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection/function.h,v $
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
 * $Log: function.h,v $
 * Revision 1.1  2002/09/14 20:44:56  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1998/09/05 00:25:26  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.6  1998/08/29 21:44:42  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.5  1998/08/23 22:57:39  fox
 * First version of building maps of humans.
 *
 * Revision 1.4  1997/06/03 11:49:14  fox
 * Museum version.
 *
 * Revision 1.3  1997/05/06 14:22:57  fox
 * Nothing special.
 *
 * Revision 1.2  1997/05/05 16:54:06  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.5  1997/01/30 17:17:23  fox
 * New version with integrated laser.
 *
 * Revision 1.4  1997/01/16 19:43:23  fox
 * And another bug ...
 *
 * Revision 1.3  1996/12/02 10:32:05  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.2  1996/11/29 15:33:37  fox
 * ok
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


#ifndef FUNCTION_INCLUDE
#define FUNCTION_INCLUDE

#include <sys/time.h>
#include <math.h>

#include "general.h"
#include "inline.h"

#ifndef PI
#define PI 3.1415926535897932384626433
#endif
#ifndef RAND_MAX
#define   RAND_MAX    32767
#endif

#define DEG_90  M_PI_2
#define DEG_180 M_PI
#define DEG_270 (M_PI + M_PI_2)
#define DEG_360 (M_PI + M_PI)


typedef struct {
  float m;
  float b;
} linFunctionParam;


int
randMax(int max);


float 
fNorm( float a, float amin, float amax, float bmin, float bmax);
  
float 
normedAngle(float angle);

/**************************************************************************
 **************************************************************************/

#define EMERGENCY_TIMER 3
#define GRAPHICS_TIMER 3
#define DETECTION_TIMER 4
#define SOUND_TIMER 5
#define SOUND_ROAR_TIMER 6
#define NO_SOUND_TIMER 7

void
setTimer( int i);

float
resetTimer( int i);

float
timeExpired( int i);

float
timeDiff( struct timeval* t1, struct timeval* t2);

/**************************************************************************
 **************************************************************************/

float
gauss(float x, float sigma, float mean);

long
round(double x);

float
Deg2Rad(float x);


float 
Sqr( float x);

float
Rad2Deg(float x);


float 
linFunction( float m, float b, float x);

void 
initFastLog();

float 
fastLog( float x);

gridPosition
gridPositionOfRealPosition( realPosition pos, int resolution);

float
positionDist( float x1, float y1, float x2, float y2);

void
movementBetweenPoints( realPosition start, realPosition end,
		       float* forward, float* sideward, float* rot);

#endif


