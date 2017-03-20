
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/function.h,v $
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
 * $Log: function.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.17  2000/03/06 20:00:43  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.16  1999/08/27 22:22:32  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.15  1999/02/17 19:42:22  fox
 * Enhanced gif utilities.
 *
 * Revision 1.14  1998/12/16 08:51:48  wolfram
 * This version does not need the libGetDistance to compute the expected Distances
 *
 * Revision 1.13  1998/11/03 21:02:17  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.12  1998/09/25 04:02:54  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.11  1998/08/23 00:00:59  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.10  1997/10/01 11:29:57  fox
 * Minor changes.
 *
 * Revision 1.9  1997/08/08 19:52:12  wolfram
 * Robot window now displays the simulator map as seen from the lasers.
 * Simulator map can be displayed at a certain ZRange.
 *
 * Revision 1.8  1997/08/02 16:51:02  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.7  1997/05/27 07:42:32  fox
 * Nothing special.
 *
 * Revision 1.6  1997/04/30 12:25:40  fox
 * Some minor changes.
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

#include <stdlib.h>
#include <sys/time.h>
#include <math.h>

#include "inline.h"
#include "map.h"

long int random(void);


#ifndef PI
#define PI 3.1415926535897932384626433
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define     M_PI_2          1.57079632679489661923 
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

typedef struct {
  int size;
  float* element;
} kernel;


int
randMax(int max);



#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_ZERO_TO_ONE() ((float)(random()) / ( RANDOM_MAX))

double
pointDistance(double x1, double y1, double x2, double y2);

float
randomGauss();

float 
fNorm( float a, float amin, float amax, float bmin, float bmax);
  
float 
normalizedAngle(float angle);


bool
intersection(float from1, float to1, float from2, float to2);


/**************************************************************************
 **************************************************************************/

#define GRAPHICS_TIMER 3
#define DETECTION_TIMER 4
#define MAP_TIMER 5
#define GIF_TIMER 6
#define MAP_CONNECTION_TIMER 7
#define PLAN_CONNECTION_TIMER 8

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
deg2Rad(float x);


float
rad2Deg(float x);


float 
linFunction( float m, float b, float x);

void 
initFastLog();

float 
fastLog( float x);

/**************************************************************************
 **************************************************************************/
void
normalize1D( probability *prob, int numberOfValues, float minVal);

void
convolve1D( probability* vector,
	    int size,
	    kernel kern);

void
convolve1DTorus( probability* vector,
	      int size,
	      kernel kern);

void
convolve2D( probability*** grid,
	    int sizeX,
	    int sizeY,
	    int sizeZ,
	    kernel xKernel,
	    kernel yKernel);

void
convolveXYPlane( probability*** grid,
		 probability minValue,
		 bool* updatePlane,
		 int sizeX,
		 int sizeY,
		 int sizeZ,
		 kernel xKernel,
		 kernel yKernel);

void
convolveZDimension( probability*** grid,
		    probability minValue,
		    bool* updatePlane,
		    int sizeX,
		    int sizeY,
		    int sizeZ,
		    kernel zKernel);

void
convolveMatrix( probability** grid,
		int sizeX,
		int sizeY,
		kernel kern);

#endif


