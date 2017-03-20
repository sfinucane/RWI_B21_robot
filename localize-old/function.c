
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/function.c,v $
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
 * $Log: function.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.25  2000/03/06 20:00:43  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.24  2000/01/02 15:33:15  fox
 * Should work.
 *
 * Revision 1.23  1999/11/02 18:12:33  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.22  1998/12/16 08:51:47  wolfram
 * This version does not need the libGetDistance to compute the expected Distances
 *
 * Revision 1.21  1998/09/25 04:02:54  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.20  1998/08/23 00:00:59  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.19  1997/11/27 18:11:17  fox
 * Several changes to make angles work better.
 *
 * Revision 1.18  1997/11/27 16:23:39  fox
 * Bug in convolveZPlane?
 *
 * Revision 1.17  1997/11/23 15:50:17  wolfram
 * Changes because of robotDump
 *
 * Revision 1.16  1997/11/20 12:58:09  fox
 * Version with good sensor selection.
 *
 * Revision 1.15  1997/10/01 11:29:57  fox
 * Minor changes.
 *
 * Revision 1.14  1997/08/08 19:52:12  wolfram
 * Robot window now displays the simulator map as seen from the lasers.
 * Simulator map can be displayed at a certain ZRange.
 *
 * Revision 1.13  1997/08/02 16:51:02  wolfram
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
 * Revision 1.12  1997/02/12 15:08:36  fox
 * Integrated laser support.
 *
 * Revision 1.11  1997/01/30 17:17:23  fox
 * New version with integrated laser.
 *
 * Revision 1.10  1997/01/29 12:23:07  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.9  1997/01/19 23:35:14  fox
 * Fixed a bug in convolve.
 *
 * Revision 1.8  1997/01/16 19:43:23  fox
 * And another bug ...
 *
 * Revision 1.7  1997/01/16 12:42:48  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.6  1996/12/02 18:46:25  fox
 * First version with the new expected distances.
 *
 * Revision 1.5  1996/12/02 10:32:05  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.4  1996/11/29 15:33:37  fox
 * ok
 *
 * Revision 1.3  1996/10/24 12:07:09  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:53  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
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


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "function.h"
#include "file.h"
#include "localTcx.h"

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



double
pointDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(fSqr(x1 - x2) + fSqr(y1 - y2));
}



/**************************************************************************
 * Norms the angle between [0:2pi].
 **************************************************************************/

float 
normalizedAngle(float angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}

float
deg2Rad(float x)
{
  return x * 0.017453293;
}


float
rad2Deg(float x)
{
  return x * 57.29578;
}


int
randMax(int max)
{
  return (int) (((float) (max + 1) * rand()) / (RAND_MAX + 1.0));
}

float
randomGauss()
{
  static int iset = 0;
  static float gset;
  float fac, rsq, v1, v2;
  if(iset == 0) {
    do {
      v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
      v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;         
      rsq = v1*v1 + v2*v2;
    } while(rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  }
  else {
    iset = 0;
    return gset;
  }
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
    return( tmp / (sqrt( 2.0 * M_PI) * sigma));
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
  if (x < 0)
    return - (long) floor(0.5 - x);
  else
    return (long) floor(x + 0.5);
}



bool
intersection(float from1, float to1, float from2, float to2){
  return from2 <= to1 && from1 <= to2;
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


/**********************************************************************/


void
normalize1D( probability *prob, int numberOfValues, float minVal){
  int i;
  double sum = 0.0;
  probability *p = prob;

  if (numberOfValues > 0){
    for (i = 0; i < numberOfValues; i++){
      sum+= *p++;
    }

    if (sum != 0.0)
      sum = 1.0 / sum;
    else 
      sum = 1.0 / numberOfValues;

    p = prob;
    for (i = 0; i < numberOfValues; i++)
      *p++ *= sum;

    /* set the minimum Probability */
    p = prob;
    sum = 0;
    for (i = 0; i < numberOfValues; i++) {
      if (*p < minVal)
	*p = minVal;
      sum += *p++;
    }

    /* normalize again */
    sum = 1/sum;
    p = prob;
    for (i = 0; i < numberOfValues; i++)
      *p++ *= sum;
    
  }
}



/**************************************************************************
 * Function for the probabilities of sonar readings.
 **************************************************************************/

#define MINIMUM_READING_PROBABILITY 0.01
#define CLOSE_SIGMA 20.0
#define FAR_SIGMA 30.0
#define CLOSE_FACTOR 8.0
#define FAR_FACTOR 10.0

void
convolve1DTorus( probability* vector, int size, kernel kern)
{
  int x, copyCnt, maxX;
  int kernCnt, kernelSize = kern.size;
  float kernelElement;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  
  /* If it is the first call to this function we allocate memory for the
   * temporary data. */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = size;
    tmpArray = (probability *) malloc( size * sizeof(probability));
  }

  /* If the size of the array has changed. */
  else if ( size > sizeOfTmpArray) {
    if ( realloc( tmpArray, size * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveFirstDimension.\n");
      return;
    }
    sizeOfTmpArray = size;
  }
  
  /* In this loop we deal with the borders of the vector. In these
   * areas not all values are defined so that we have to check for
   * range errors and consider this in the norm factor <kernelSum>. */
  for ( x = 0; x < size; x++) {
    
    /* Jump to the end of the data. */
    if ( x == (kernelSize - 1))
      x = size - kernelSize + 1;
    
    /* The middle of the kernel. */
    tmpArray[x] = kern.element[0] * vector[x];
    
    /* Store the weighted sum in the tmpArray. */
    for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {
      
      kernelElement = kern.element[kernCnt];
      
      if ( x + kernCnt < size) 
	tmpArray[x] += kernelElement * vector[x + kernCnt];
      else
	tmpArray[x] += kernelElement * vector[x + kernCnt - size];
      
      if (x - kernCnt >= 0) 
	tmpArray[x] += kernelElement * vector[x - kernCnt];
      else
	tmpArray[x] += kernelElement * vector[x - kernCnt + size];
    }
  }
  
  /* Now we can compute the weighted sums without range checks. */
  maxX = size-kernelSize+1;
  for ( x = kernelSize - 1; x < maxX; x++) {

    /* The middle of the kernel. */
    tmpArray[x] = kern.element[0] * vector[x];
    
    /* Store the weighted kernelSum in the tmpArray. */
    for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
      tmpArray[x] += kern.element[kernCnt]
	* (vector[x+kernCnt] + vector[x-kernCnt]);
  }
  
  /* Now copy the values from the tmpArray back into the vector. */
  for ( copyCnt = 0; copyCnt < size; copyCnt++) 
    vector[copyCnt] = tmpArray[copyCnt];
}

void
convolve1D( probability* vector, int size, kernel kern)
{
  int x, copyCnt, maxX;
  int kernCnt, kernelSize = kern.size;
  float kernelElement;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  
  /* If it is the first call to this function we allocate memory for the
   * temporary data. */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = size;
    tmpArray = (probability *) malloc( size * sizeof(probability));
  }

  /* If the size of the array has changed. */
  else if ( size > sizeOfTmpArray) {
    if ( realloc( tmpArray, size * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveFirstDimension.\n");
      return;
    }
    sizeOfTmpArray = size;
  }
  
  /* In this loop we deal with the borders of the vector. In these
   * areas not all values are defined so that we have to check for
   * range errors and consider this in the norm factor <kernelSum>. */
  for ( x = 0; x < size; x++) {

    probability weight = kern.element[0];
    
    /* Jump to the end of the data. */
    if ( x == (kernelSize - 1))
      x = size - kernelSize + 1;
    
    /* The middle of the kernel. */
    tmpArray[x] = kern.element[0] * vector[x];
    
    /* Store the weighted sum in the tmpArray. */
    for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {
      
      kernelElement = kern.element[kernCnt];
      
      if ( x + kernCnt < size) {
	weight += kernelElement;
	tmpArray[x] += kernelElement * vector[x + kernCnt];
      }
      
      if (x - kernCnt >= 0) {
	weight += kernelElement;
	tmpArray[x] += kernelElement * vector[x - kernCnt];
      }
    }
    if ( weight != 0.0)
      tmpArray[x] /= weight;
  }

  /* Now we can compute the weighted sums without range checks. */
  maxX = size-kernelSize+1;
  for ( x = kernelSize - 1; x < maxX; x++) {

    /* The middle of the kernel. */
    tmpArray[x] = kern.element[0] * vector[x];
    
    /* Store the weighted kernelSum in the tmpArray. */
    for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
      tmpArray[x] += kern.element[kernCnt]
	* (vector[x+kernCnt] + vector[x-kernCnt]);
  }
  
  /* Now copy the values from the tmpArray back into the vector. */
  for ( copyCnt = 0; copyCnt < size; copyCnt++) 
    vector[copyCnt] = tmpArray[copyCnt];
}


static void
convolveFirstDimension( probability*** grid,
			int sizeX,
			int sizeY,
			int sizeZ,
			kernel kern)
{
  int x, y, z, copyCnt, maxX;
  int kernCnt, kernelSize = kern.size;
  probability kernelSum;
  float kernelElement;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  
  /* If it is the first call to this function we allocate memory for the
   * temporary data. */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = sizeX;
    tmpArray = (float *) malloc( sizeX * sizeof(probability));
  }

  /* If the size of the array has changed. */
  else if ( sizeX > sizeOfTmpArray) {
    if ( realloc( tmpArray, sizeX * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveFirstDimension.\n");
      return;
    }
    sizeOfTmpArray = sizeX;
  }
  
  /* Shift the kernel over the x-dimension. */
  for ( y = 0; y < sizeY; y++) 
    for ( z = 0; z < sizeZ; z++) {
      swallowStatusReports(DONT_WAIT);
      /* In this loop we deal with the borders of the grid. In these
       * areas not all values are defined so that we have to check for
       * range errors and consider this in the norm factor <kernelSum>. */
      for ( x = 0; x < sizeX; x++) {

	/* Jump to the end of the data. */
	if ( x == (kernelSize - 1))
	  x = sizeX - kernelSize + 1;

	/* The middle of the kernel. */
	tmpArray[x] = kern.element[0] * grid[z][x][y];
	kernelSum = kern.element[0];
	
	/* Store the weighted kernelSum in the tmpArray. */
	for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {

	  kernelElement = kern.element[kernCnt];
	  
	  if ( x + kernCnt < sizeX) {
	    tmpArray[x] += kernelElement * grid[z][x+kernCnt][y];
	    kernelSum         += kernelElement;
	  }
	  if (x - kernCnt >= 0) {
	    tmpArray[x] += kernelElement * grid[z][x-kernCnt][y];
	    kernelSum         += kernelElement;
	  }
	}
	/* Norm the value. */
	if ( kernelSum > 0.0)
	  tmpArray[x] /= kernelSum;
      }

      /* Now we can compute the weighted sums without range checks. */
      maxX = sizeX-kernelSize+1;
      for ( x = kernelSize - 1; x < maxX; x++) {
	
	/* The middle of the kernel. */
	tmpArray[x] = kern.element[0] * grid[z][x][y];
	
	/* Store the weighted kernelSum in the tmpArray. */
	for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
	  tmpArray[x] += kern.element[kernCnt]
	    * (grid[z][x+kernCnt][y] + grid[z][x-kernCnt][y]);
      }
      /* Now copy the values from the tmpArray back into the grid. */
      for ( copyCnt = 0; copyCnt < sizeX; copyCnt++)
	grid[z][copyCnt][y] = tmpArray[copyCnt];
    }
}

static void
convolveSecondDimension( probability*** grid,
			 int sizeX,
			 int sizeY,
			 int sizeZ,
			 kernel kern)
{
  int x, y, z, copyCnt, maxY;
  int kernCnt, kernelSize = kern.size;
  probability kernelSum;
  float kernelElement;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  
  /* If it is the first call to this function we allocate memory for the
   * temporary data.
   */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = sizeY;
    tmpArray = (float *) malloc( sizeY * sizeof(probability));
  }
  /* If the size of the array is not big enough.
   */
  else if ( sizeY > sizeOfTmpArray) {
    if ( realloc( tmpArray, sizeY * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveSecondDimension.\n");
      return;
    }
    sizeOfTmpArray = sizeY;
  }
  
  /* Shift the kernel over the y-dimension. */
  for ( x = 0; x < sizeX; x++) 
    for ( z = 0; z < sizeZ; z++) {
      swallowStatusReports(DONT_WAIT);
      /* In this loop we just deal with the borders of the grid. In these
       * areas not all values are defined. */
      for ( y = 0; y < sizeY; y++) {

	/* Jump to the end of the data. */
	if ( y == (kernelSize - 1))
	  y = sizeY - kernelSize + 1;

	/* The middle of the kernel. */
	tmpArray[y] = kern.element[0] * grid[z][x][y];
	kernelSum = kern.element[0];
	
	/* Store the weighted kernelSum in the tmpArray. */
	for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {

	  kernelElement = kern.element[kernCnt];
	  
	  if ( y + kernCnt < sizeY) {
	    tmpArray[y] += kernelElement * grid[z][x][y+kernCnt];
	    kernelSum         += kernelElement;
	  }
	  if (y - kernCnt >= 0) {
	    tmpArray[y] += kernelElement * grid[z][x][y-kernCnt];
	    kernelSum         += kernelElement;
	  }
	}
	/* Norm the value. */
	if ( kernelSum > 0.0)
	  tmpArray[y] /= kernelSum;
      }

      /* Now we can compute the weighted sums without range checks. */
      maxY = sizeY-kernelSize+1;
      for ( y = kernelSize - 1; y < maxY; y++) {
	
	/* The middle of the kernel. */
	tmpArray[y] = kern.element[0] * grid[z][x][y];
	
	/* Store the weighted kernelSum in the tmpArray. */
	for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
	  tmpArray[y] += kern.element[kernCnt]
	    * (grid[z][x][y+kernCnt] + grid[z][x][y-kernCnt]);
      }
      /* Now copy the values from the tmpArray back into the grid. */
      for ( copyCnt = 0; copyCnt < sizeY; copyCnt++)
	grid[z][x][copyCnt] = tmpArray[copyCnt];
    }
}

static void
convolveThirdDimension( probability*** grid,
			int sizeX,
			int sizeY,
			int sizeZ,
			kernel kern)
{
  int x, y, z, copyCnt, maxZ;
  int kernCnt, kernelSize = kern.size;
  probability kernelSum;
  float kernelElement;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  
  /* If it is the first call to this function we allocate memory for the
   * temporary data.
   */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = sizeZ;
    tmpArray = (float *) malloc( sizeZ * sizeof(probability));
  }
  /* If the size of the array is not big enough.
   */
  else if ( sizeZ > sizeOfTmpArray) {
    if ( realloc( tmpArray, sizeZ * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveThirdDimension.\n");
      return;
    }
    sizeOfTmpArray = sizeZ;
  }
  
  /* Shift the kernel over the z-dimension. */
  for ( x = 0; x < sizeX; x++) 
    for ( y = 0; y < sizeY; y++) {
      swallowStatusReports(DONT_WAIT);
      /* In this loop we just deal with the borders of the grid. In these
       * areas not all values are defined. */
      for ( z = 0; z < sizeZ; z++) {

	/* Jump to the end of the data. */
	if ( z == (kernelSize - 1))
	  z = sizeZ - kernelSize + 1;

	/* The middle of the kernel. */
	tmpArray[z] = kern.element[0] * grid[z][x][y];
	kernelSum = kern.element[0];
	
	/* Store the weighted sum in the tmpArray. */
	for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {

	  kernelElement = kern.element[kernCnt];
	  
	  if ( z + kernCnt < sizeZ) {
	    tmpArray[z] += kernelElement * grid[z+kernCnt][x][y];
	    kernelSum         += kernelElement;
	  }
	  if (z - kernCnt >= 0) {
	    tmpArray[z] += kernelElement * grid[z-kernCnt][x][y];
	    kernelSum         += kernelElement;
	  }
	}
	/* Norm the value. */
	if ( kernelSum > 0.0) 
	  tmpArray[z] /= kernelSum; 
      }

      /* Now we can compute the weighted sums without range checks. */
      maxZ = sizeZ-kernelSize+1;
      for ( z = kernelSize - 1; z < maxZ; z++) {
	
	/* The middle of the kernel. */
	tmpArray[z] = kern.element[0] * grid[z][x][y];
	
	/* Store the weighted kernelSum in the tmpArray. */
	for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
	  tmpArray[z] += kern.element[kernCnt]
	    * (grid[z+kernCnt][x][y] + grid[z-kernCnt][x][y]);
      }
      /* Now copy the values from the tmpArray back into the grid. */
      for ( copyCnt = 0; copyCnt < sizeZ; copyCnt++)
	grid[copyCnt][x][y] = tmpArray[copyCnt];
    }
}

static int
extractDefinedBlocksFirstDimension( probability*** grid,
				    probability minValue,
				    int sizeX,
				    int y,
				    int z,
				    int kernelDistance,
				    int* blockBegin,
				    int* blockEnd)
{
  int x, firstUnknownX = 0;
  int blockNumber = 0;
  
  probability **gridPlane = grid[z];
  
  do {
    
    /* Look for the next defined value. */
    for ( x = firstUnknownX;
	  x < sizeX;
		x++) 
      if ( gridPlane[x][y] >= minValue) { 
	blockBegin[blockNumber] = iMax( firstUnknownX, x - kernelDistance);
	break;
      }

    /* No defined value found. */
    if ( x == sizeX)
      return blockNumber;
	    
    /* Look for the next unDefined value. */
    for ( ++x;
	  x < sizeX;
	  x++) 
      if ( gridPlane[x][y] < minValue) {
	firstUnknownX = x + 1;
	blockEnd[blockNumber] = iMin( sizeX, x + kernelDistance);
	break;
      }
    
    /* No undefined value found. */
    if ( x == sizeX) {
      blockEnd[blockNumber] = sizeX;
      return blockNumber+1;
    }
    
    blockNumber++;
  }
  while( x <= sizeX);

  fprintf(stderr, "Shouldn't come to this point!\n");
  return 0;
}


static void
convolveUpdatePlanesFirstDimension( probability*** grid,
				   probability minValue,
				   bool* updatePlane,
				   int sizeX,
				   int sizeY,
				   int sizeZ,
				   kernel kern)
{
  int x, y, z;
  probability kernelSum;
  float kernelElement;
  int kernCnt, kernelSize = kern.size;
  int maxKernelDistance = kern.size - 1;
  int cnt = 0;
  int numberOfBlocks, blockNumber;

  probability **gridPlane;
  
  static int* blockBegin;
  static int* blockEnd;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  /* If it is the first call to this function we allocate memory for the
   * temporary data. */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = sizeX;
    tmpArray = (float *) malloc( sizeX * sizeof(probability));
    blockBegin = (int*) malloc( sizeX / 2 * sizeof( int));
    blockEnd = (int*) malloc( sizeX / 2 * sizeof( int));
    if ( tmpArray == NULL || blockBegin == NULL || blockEnd == NULL) {
      fprintf(stderr,"ERROR: Not enough memory in convolve first dimension.\n");
      return;
    }
  }
  /* If the size of the array has changed. */
  else if ( sizeX > sizeOfTmpArray) {
    if ( realloc( tmpArray, sizeX * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveFirstDimension.\n");
      return;
    }
    sizeOfTmpArray = sizeX;
  }

  /* Shift the kernel over the x-dimension. */
  for ( z = 0; z < sizeZ; z++)

    /* Only consider planes with values high enough. */
    if ( updatePlane[z]) {
      gridPlane = grid[z];

      for ( y = 0; y < sizeY; y++)  {

	numberOfBlocks = extractDefinedBlocksFirstDimension( grid,
							     minValue,
							     sizeX, y, z,
							     maxKernelDistance,
							     blockBegin, blockEnd);

	for ( blockNumber = 0; blockNumber < numberOfBlocks; blockNumber++) {

	  for ( x = blockBegin[blockNumber];
		x < blockEnd[blockNumber]; x++) {

	    cnt++;

	    /* Here we deal with the borders of the grid. In these
	     * areas not all values are defined so that we have to check for
	     * range errors and consider this in the norm factor <kernelSum>. */
	    if ( x < maxKernelDistance || x >= sizeX - maxKernelDistance) {
	    
	      /* The middle of the kernel. */
	      tmpArray[x] = kern.element[0] * gridPlane[x][y];
	      kernelSum = kern.element[0];
	      
	      /* Store the weighted kernelSum in the tmpArray. */
	      for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {
		
		kernelElement = kern.element[kernCnt];
		
		if ( x + kernCnt < sizeX) {
		  tmpArray[x] += kernelElement * gridPlane[x+kernCnt][y];
		  kernelSum         += kernelElement;
		}
		if (x - kernCnt >= 0) {
		  tmpArray[x] += kernelElement * gridPlane[x-kernCnt][y];
		  kernelSum         += kernelElement;
		}
	      }
	      /* Norm the value. */
	      if ( kernelSum > 0.0)
		tmpArray[x] /= kernelSum;
	    }
	    else {
	      
	      /* The middle of the kernel. */
	      tmpArray[x] = kern.element[0] * gridPlane[x][y];

	      /* Store the weighted kernelSum in the tmpArray. */
	      for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
		tmpArray[x] += kern.element[kernCnt]
		  * (gridPlane[x+kernCnt][y] + gridPlane[x-kernCnt][y]);
	    }
	  }
	}

	/* Now copy the values from the tmpArray back into the grid. */
	for ( blockNumber = 0; blockNumber < numberOfBlocks; blockNumber++) 
	  for ( x = blockBegin[blockNumber]; x < blockEnd[blockNumber]; x++) 
	    gridPlane[x][y] = tmpArray[x];
      }
    }
/*     else */
/*       printf( "dont convolve plane %d\n", z); */
  
    writeLog( "(%.2f, ", 100.0 * (float) (cnt) /
	    (float) (sizeX * sizeY * sizeZ ));
}


static int
extractDefinedBlocksSecondDimension( probability*** grid,
				     probability minValue,
				     int sizeY,
				     int x,
				     int z,
				     int kernelDistance,
				     int* blockBegin,
				     int* blockEnd)
{
  int y, firstUnknownY = 0;
  int blockNumber = 0;

  probability *gridRow = grid[z][x];
  do {
    
    /* Look for the next defined value. */
    for ( y = firstUnknownY;
	  y < sizeY;
		y++) 
      if ( gridRow[y] >= minValue) { 
	blockBegin[blockNumber] = iMax( firstUnknownY, y - kernelDistance);
	break;
      }

    /* No defined value found. */
    if ( y == sizeY)
      return blockNumber;
	    
    /* Look for the next unDefined value. */
    for ( ++y;
	  y < sizeY;
	  y++) 
      if ( gridRow[y] < minValue) {
	firstUnknownY = y + 1;
	blockEnd[blockNumber] = iMin( sizeY, y + kernelDistance);
	break;
      }
    
    /* No undefined value found. */
    if ( y == sizeY) {
      blockEnd[blockNumber] = sizeY;
      return blockNumber+1;
    }
    
    blockNumber++;
  }
  while( y <= sizeY);

  fprintf(stderr, "Shouldn't come to this point!\n");
  return 0;
}


static void
convolveUpdatePlanesSecondDimension( probability*** grid,
				    probability minValue,
				    bool* updatePlane,
				    int sizeX,
				    int sizeY,
				    int sizeZ,
				    kernel kern)
{
  int x, y, z;
  probability kernelSum;
  float kernelElement;
  int kernCnt, kernelSize = kern.size;
  int maxKernelDistance = kern.size - 1;
  int cnt = 0;
  int numberOfBlocks, blockNumber;

  probability *gridRow;
  
  static int* blockBegin;
  static int* blockEnd;
  
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  /* If it is the first call to this function we allocate memory for the
   * temporary data. */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = sizeY;
    tmpArray = (float *) malloc( sizeY * sizeof(probability));
    blockBegin = (int*) malloc( sizeY / 2 * sizeof( int));
    blockEnd = (int*) malloc( sizeY / 2 * sizeof( int));
    if ( tmpArray == NULL || blockBegin == NULL || blockEnd == NULL) {
      fprintf(stderr,"ERROR: Not enough memory in convolve second dimension.\n");
      return;
    }
  }
  /* If the size of the array has changed. */
  else if ( sizeY > sizeOfTmpArray) {
    if ( realloc( tmpArray, sizeY * sizeof(probability)) == NULL) {
      fprintf( stderr, "ERROR in reallocating tmpArray in convolveFirstDimension.\n");
      return;
    }
    sizeOfTmpArray = sizeY;
  }

  /* Shift the kernel over the y-dimension. */
  for ( z = 0; z < sizeZ; z++)
    
    /* Only consider planes with values high enough. */
    if ( updatePlane[z]) 

      for ( x = 0; x < sizeX; x++)  {

	numberOfBlocks = extractDefinedBlocksSecondDimension( grid,
							     minValue,
							     sizeY, x, z,
							     maxKernelDistance,
							     blockBegin, blockEnd);

	gridRow = grid[z][x];
	
	for ( blockNumber = 0; blockNumber < numberOfBlocks; blockNumber++) {

	  for ( y = blockBegin[blockNumber];
		y < blockEnd[blockNumber]; y++) {
	    
	    cnt++;
	    
	    /* Here we deal with the borders of the grid. In these
	     * areas not all values are defined so that we have to check for
	     * range errors and consider this in the norm factor <kernelSum>. */
	    if ( y < maxKernelDistance || y >= sizeY - maxKernelDistance) {
	      
	      /* The middle of the kernel. */
	      tmpArray[y] = kern.element[0] * gridRow[y];
	      kernelSum = kern.element[0];
	      
	      /* Store the weighted kernelSum in the tmpArray. */
	      for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {
		
		kernelElement = kern.element[kernCnt];
		
		if ( y + kernCnt < sizeY) {
		  tmpArray[y] += kernelElement * gridRow[y+kernCnt];
		  kernelSum         += kernelElement;
		}
		if (y - kernCnt >= 0) {
		  tmpArray[y] += kernelElement * gridRow[y-kernCnt];
		  kernelSum         += kernelElement;
		}
	      }
	      /* Norm the value. */
	      if ( kernelSum > 0.0)
		tmpArray[y] /= kernelSum;
	    }
	    else {
	      
	      /* The middle of the kernel. */
	      tmpArray[y] = kern.element[0] * gridRow[y];
	      
	      /* Store the weighted kernelSum in the tmpArray. */
	      for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
		tmpArray[y] += kern.element[kernCnt]
		  * (gridRow[y+kernCnt] + gridRow[y-kernCnt]);
	    }
	  }
	}
	
	/* Now copy the values from the tmpArray back into the grid. */
	for ( blockNumber = 0; blockNumber < numberOfBlocks; blockNumber++) 
	  for ( y = blockBegin[blockNumber]; y < blockEnd[blockNumber]; y++) 
	    gridRow[y] = tmpArray[y];
      }
  
  writeLog( "%.2f ", 100.0 * (float) (cnt) /
	   (float) (sizeX * sizeY * sizeZ ));
}


static void
convolveUpdatePlanesThirdDimension( probability*** grid,
				    probability minValue,
				    bool* updatePlane,
				    int sizeX,
				    int sizeY,
				    int sizeZ,
				    kernel kern)
{
  int x, y, z, z1, cnt = 0;
  int kernCnt, kernelSize = kern.size;
  int numberOfBlocks, blockNumber;
  int offset = kernelSize - 1;
  register probability center;
  int firstPlane = 0, lastPlane = sizeZ - 1, endPlane;
  bool convolve;
  
  /* Only allocate memory if necessary. */
  static bool firstTime = TRUE;
  static int sizeOfTmpArray = 0;
  static probability* tmpArray;
  static int* blockBegin;
  static int* blockEnd;
  
  /* If it is the first call to this function we allocate memory for the
   * temporary data.
   */
  if (firstTime) {
    firstTime = FALSE;
    sizeOfTmpArray = sizeZ + 2*offset;
    tmpArray = (probability *) malloc( sizeOfTmpArray * sizeof(probability));
    blockBegin = (int*) malloc( sizeX / 2 * sizeof( int));
    blockEnd = (int*) malloc( sizeX / 2 * sizeof( int));
    if ( tmpArray == NULL || blockBegin == NULL || blockEnd == NULL) {
      fprintf(stderr,"ERROR: Not enough memory in convolve third dimension.\n");
      return;
    }
  }
 
  /* If the size of the array is not big enough.
   */
  else if ( sizeZ + 2*offset> sizeOfTmpArray) {
    if ( realloc( tmpArray, (sizeZ + 2*offset) *
		  sizeof(probability)) == NULL) {
      fprintf( stderr,
	       "ERROR in reallocating tmpArray in convolveThirdDimension.\n");
      return;
    }
    sizeOfTmpArray = sizeZ + 2*offset;
  }

  while (firstPlane < sizeZ && !updatePlane[firstPlane])
    firstPlane++;
  while (lastPlane > 0 && !updatePlane[lastPlane])
    lastPlane--;

  endPlane = lastPlane + 1;
  
  /* find plane blocks */
  numberOfBlocks = 0;
  z = firstPlane;
  do {
    /* find begin of block */
    while (z < endPlane && !updatePlane[z])
      z++;
    
    if (z < endPlane){
      /* find end of block */
      blockBegin[numberOfBlocks] = z;
      do {
	z++;
      }
      while (z < endPlane && updatePlane[z]);
      blockEnd[numberOfBlocks++] = z;
      z++;
    }
  }
  while (z < endPlane);

  for (z = 0; z < sizeOfTmpArray; z++)
    tmpArray[z] = minValue;

  /* Shift the kernel over the y-dimension. */
  for ( x = 0; x < sizeX; x++) 
    for ( y = 0; y < sizeY; y++) {
       
      /* fill in the borders for convolving in one round */
      z=sizeZ-offset;
      for (z1 = 0; z1 < offset; z1++,z++)
	if ( updatePlane[z])
	  tmpArray[z1] = grid[z][x][y];
	else
	  tmpArray[z1] = minValue;
       z=0;
       for (z1 = sizeZ + offset; z1 < sizeZ + offset + offset; z1++,z++)
	 if ( updatePlane[z])
	   tmpArray[z1] = grid[z][x][y];
	 else
	   tmpArray[z1] = minValue;
       
       
       /* Now we can compute the weighted sums without any checks. */       
       for ( blockNumber = 0; blockNumber < numberOfBlocks; blockNumber++){

	 convolve = FALSE;
	 z1 = blockBegin[blockNumber] + offset;
	 for (z = blockBegin[blockNumber]; z < blockEnd[blockNumber]; z++){
	   if ((tmpArray[z1++] = grid[z][x][y]) > minValue) convolve = TRUE;
	 }

	 if (convolve){

	   z1 = blockBegin[blockNumber] + offset;
	   for (z = blockBegin[blockNumber]; z < blockEnd[blockNumber]; z++){
	     cnt++;

	     /* The middle of the kernel. */
	     center = kern.element[0] * tmpArray[z1];
	     
	     /* Store the weighted kernelSum in the tmpArray. */
	     for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++)
	       center += kern.element[kernCnt]
		 * (tmpArray[z1+kernCnt] + tmpArray[z1-kernCnt]);
	     
	     /* Now copy the values from the tmpArray back into the grid. */
	     grid[z][x][y] = center;
	     z1++;
	   }
	 }
       }
    }

  writeLog( "%.2f)", 100.0 * (float) (cnt) / (float) (sizeX * sizeY * sizeZ ));
  
  writeLog("corrected Blocks: ");
  for (blockNumber = 0; blockNumber < numberOfBlocks; blockNumber++){
          writeLog("%d - %d ",blockBegin[blockNumber],
	      blockEnd[blockNumber]);
  }
  writeLog("\n");

}


void
convolve2D( probability*** grid,
	    int sizeX,
	    int sizeY,
	    int sizeZ,
	    kernel xKernel,
	    kernel yKernel)
{
  convolveFirstDimension( grid,
			  sizeX,
			  sizeY,
			  sizeZ,
			  xKernel);
		 
  convolveSecondDimension( grid,
			   sizeX,
			   sizeY,
			   sizeZ,
			   yKernel);
}

void
convolveMatrix( probability** grid,
		int sizeX,
		int sizeY,
		kernel smoothKernel)
{
  probability*** g = &grid;
  
  convolveSecondDimension( g,
			   1,
			   sizeX,
			   sizeY,
			   smoothKernel);
  
  convolveThirdDimension( g,
			  1,
			  sizeX,
			  sizeY,
			  smoothKernel);
}



void
convolveXYPlane( probability*** grid,
		 probability minValue,
		 bool* updatePlane,
		 int sizeX,
		 int sizeY,
		 int sizeZ,
		 kernel xKernel,
		 kernel yKernel)
{
  if ( xKernel.size < 2) 
    fprintf( stderr, "Warning: x Kernel size must be at least 2!\n");
  else
    convolveUpdatePlanesFirstDimension( grid,
					minValue,
					updatePlane,
					sizeX,
					sizeY,
					sizeZ,
					xKernel);
  
  if ( yKernel.size < 2) 
    fprintf( stderr, "Warning: y Kernel size must be at least 2!\n");
  else
    convolveUpdatePlanesSecondDimension( grid,
					 minValue,
					 updatePlane,
					 sizeX,
					 sizeY,
					 sizeZ,
					 xKernel);
  
}

void
convolveZDimension( probability*** grid,
		    probability minValue,
		    bool* updatePlane,
		    int sizeX,
		    int sizeY,
		    int sizeZ,
		    kernel zKernel)
{
  if ( zKernel.size < 2) {
    fprintf( stderr, "Warning: z Kernel size must be at least 2!\n");
    return;
  }
    
  convolveUpdatePlanesThirdDimension( grid,
				      minValue,
				      updatePlane,
				      sizeX,
				      sizeY,
				      sizeZ,
				      zKernel);
}



