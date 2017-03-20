
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old/function.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:00 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: function.c,v $
 * Revision 1.1  2002/09/14 20:45:00  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1998/09/05 00:25:26  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.5  1998/08/23 22:57:39  fox
 * First version of building maps of humans.
 *
 * Revision 1.4  1997/05/09 16:28:39  fox
 * Works quiet fine.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "function.h"
#include "laser.h"

/* Time struct for setStartTime() and timeExpired(). */
#define MAX_NUMBER_OF_TIMERS 10
static struct timeval startTime[MAX_NUMBER_OF_TIMERS];

/*****************************************************************************/
/*   float-Normierung: b = norm (a,...)                                      */
/*****************************************************************************/
float 
fNorm( float a, float amin, float amax, float bmin, float bmax)
{
  if (amin==amax) {
    fprintf( stderr, "Warning: cannot norm from empty interval.\n");
    return( a < amin ? bmin : bmax);
  }
  else
    if (bmin==bmax)
      return(bmax);
    else
      return( (a-amin) / (amax-amin) * (bmax-bmin) + bmin);
}


/**************************************************************************
 * Norms the angle between [0:2pi].
 **************************************************************************/

float 
normedAngle(float angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}

float
Deg2Rad(float x)
{
  return x * 0.017453293;
}


float
Rad2Deg(float x)
{
  return x * 57.29578;
}


int
randMax(int max)
{
  return (int) (((float) (max + 1) * rand()) / (RAND_MAX + 1.0));
}

/**************************************************************************
 **************************************************************************
 * Some function for time measurements.
 **************************************************************************
 **************************************************************************/
void
setTimer( int i)
{
  if ( i < MAX_NUMBER_OF_TIMERS)
    gettimeofday( &(startTime[i]), 0);
  else
    fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
}


float
resetTimer( int i)
{
  if ( i < MAX_NUMBER_OF_TIMERS) {
    float expired = timeExpired( i);
    gettimeofday( &(startTime[i]), 0);
    return expired;
  }
  else {
    fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
    return 0.0;
  }
}


float
timeExpired( int i)
{
  static struct timeval now;
   
  if ( i < MAX_NUMBER_OF_TIMERS) {
    gettimeofday( &now, 0);
    return timeDiff( &now, &(startTime[i]));
  }
  else {
    fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
    return 0.0;
  }
}



float
timeDiff( struct timeval* t1, struct timeval* t2)
{
  float diff;
   
  diff =  (float) (t1->tv_usec - t2->tv_usec) / 1000000.0;
  diff += (float) (t1->tv_sec - t2->tv_sec);

  return diff;
}

	      
/**************************************************************************
 * Computes the gaussian with standard deviation <sigma> and mean <mean>.
 **************************************************************************/
float
gauss( float x, float sigma, float mean)
{
  float tmp;
    
  if (sigma != 0.0) {
    tmp = exp( - 0.5 * fSqr((mean-x) / sigma));
    return( tmp / sqrt( 2.0 * M_PI) / sigma);
  }
  else {
    if (x == mean)
      return 1.0;
    else
      return 0.0;
  }
    
}

long
round(double x) {
  return (long) floor(x + 0.5);
}


/**************************************************************************
 * Computes the linear function given the parameters of the function and x
 **************************************************************************/
float 
linFunction( float m, float b, float x) 
{
  return m * x + b;
}


/**********************************************************************
 **********************************************************************
 * Preprocessed logarithm.
 **********************************************************************
 **********************************************************************/
#define LOG_TABLE_SIZE 1000000
#define LOG_THRESHOLD  0.1

static float logTable[LOG_TABLE_SIZE+1];
static float scaleFactor;
static double invertedScale;

/**********************************************************************/
void 
initFastLog()
{
  int i;
  double x;

  scaleFactor = 1.0 / LOG_THRESHOLD * (float) LOG_TABLE_SIZE;
  invertedScale = 1.0 / (double) scaleFactor;
  
  for (i = 1; i < LOG_TABLE_SIZE; i++){
    x = ((double) i) * (double) invertedScale;
    logTable[i] = (float) log(x);
  }
}

/**********************************************************************/
float 
fastLog( float x)
{
  if ( x > LOG_THRESHOLD || x < invertedScale)
    return log(x);
  else {
/*     fprintf(stderr, "%d %f\n", (int) ( x * scaleFactor), logTable[(int) ( x * scaleFactor)]); */
    return( logTable[(int) ( x * scaleFactor)]);
  }
}

gridPosition
gridPositionOfRealPosition( realPosition pos, int resolution)
{
  gridPosition gridPos;
  
  gridPos.x = pos.x / resolution;
  gridPos.y = pos.y / resolution;
  gridPos.rot = Rad2Deg( normedAngle( pos.rot));

  if ( gridPos.x >= sizeOfEnvironmentX)
    gridPos.x = sizeOfEnvironmentX - 1;
  else if ( gridPos.x < 0)
    gridPos.x = 0;
    
  if ( gridPos.y >= sizeOfEnvironmentY)
    gridPos.y = sizeOfEnvironmentY - 1;
  else if ( gridPos.y < 0)
    gridPos.y = 0;
    
  return gridPos;
}

float
positionDist( float x1, float y1, float x2, float y2)
{

  return sqrt( fSqr( x1 - x2) + fSqr( y1 - y2));
}
     
float 
Sqr( float x)
{
  return x * x;
}


/**************************************************************************
 * Computes the movement between two points in the correct coordinate system.
 **************************************************************************/
void
movementBetweenPoints( realPosition start, realPosition end,
		       float* forward, float* sideward, float* rot)
{
  

  /* compute forward and sideward sensing_MOVEMENT */
  *forward =
    + (end.y - start.y) * sin(start.rot)
    + (end.x - start.x) * cos(start.rot);
  
  *sideward =
    + (end.y - start.y) * cos(start.rot)
    - (end.x - start.x) * sin(start.rot);

  *rot = end.rot - start.rot;
    
}

