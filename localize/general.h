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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/general.h,v $
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
 * $Log: general.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.60  2000/02/07 12:04:28  wolfram
 * Increased the number of simulator objects
 *
 * Revision 1.59  2000/01/19 16:21:47  fox
 * Version after I created density sample sets. Removed all defines so
 * that it should work properly. If you want to use zlib, set USE_ZLIB
 * in the Makefile.
 *
 * Revision 1.58  2000/01/10 19:04:20  fox
 * DON'T USE!
 *
 * Revision 1.57  1999/09/03 13:43:35  fox
 * Nothing special.
 *
 * Revision 1.56  1999/09/01 00:02:56  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.55  1999/07/13 23:08:01  fox
 * Some changes.
 *
 * Revision 1.54  1999/06/25 19:48:11  fox
 * Minor changs for the urbie.
 *
 * Revision 1.53  1999/06/24 00:21:50  fox
 * Some changes for the urbies.
 *
 * Revision 1.52  1999/04/21 22:58:00  fox
 * First attempt to get samples from multi back.
 *
 * Revision 1.51  1999/03/01 17:44:28  wolfram
 * Added support for Pioneer II.  Pioneer I will be added next
 *
 * Revision 1.50  1999/01/11 19:47:49  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.49  1998/12/16 08:51:48  wolfram
 * This version does not need the libGetDistance to compute the expected Distances
 *
 * Revision 1.48  1998/11/17 23:26:19  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.47  1998/11/03 21:02:18  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.46  1998/10/29 03:45:00  fox
 * Nothing special.
 *
 * Revision 1.45  1998/10/02 15:16:37  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.44  1998/09/25 17:53:29  fox
 * Improved version of condensation.
 *
 * Revision 1.43  1998/09/25 04:02:54  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.42  1998/09/19 13:07:44  wolfram
 * Added missing files
 *
 * Revision 1.41  1998/09/18 17:24:43  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.40  1998/08/23 00:00:59  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.39  1998/08/20 00:22:57  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.38  1998/06/12 10:16:29  fox
 * Implemented virutal sensor.
 *
 * Revision 1.37  1998/04/19 10:40:35  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.36  1998/03/17 08:41:12  wolfram
 * First steps to add vision.
 *
 * Revision 1.35  1998/03/04 14:46:44  fox
 * This version should run.
 *
 * Revision 1.34  1998/02/12 15:49:10  derr
 * librawData support implemented.
 * see Makefile for further information.
 *
 * Revision 1.33  1998/01/29 16:46:50  fox
 * Removed some hacks.
 *
 * Revision 1.32  1998/01/22 13:06:14  fox
 * First version after selection-submission.
 *
 * Revision 1.31  1997/12/19 10:30:31  wolfram
 * Changed MINIMUM_UPDATE_PROBABILITY_QUOTA and resetNewPlanes
 *
 * Revision 1.30  1997/12/17 16:48:29  wolfram
 * Added NEIGHBOR_PLANES_TO_ADD keyword for normalize
 *
 * Revision 1.29  1997/11/28 13:34:35  fox
 * Added questions.
 *
 * Revision 1.28  1997/11/23 15:50:17  wolfram
 * Changes because of robotDump
 *
 * Revision 1.27  1997/11/20 12:58:10  fox
 * Version with good sensor selection.
 *
 * Revision 1.26  1997/10/01 11:29:58  fox
 * Minor changes.
 *
 * Revision 1.25  1997/09/29 10:45:22  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.24  1997/08/16 21:52:02  wolfram
 * Grid maps can now be cad maps.  Obstacles are enlarged accordingly
 *
 * Revision 1.23  1997/08/02 16:51:02  wolfram
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
 * Revision 1.22  1997/06/26 11:23:16  fox
 * Fixed a bug in normalize.
 *
 * Revision 1.21  1997/04/27 15:48:20  wolfram
 * Changes in script.c
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
 * First version of restructured LOCALIZE.
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


#ifndef GENERAL_INCLUDE
#define GENERAL_INCLUDE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <sys/time.h>

#ifdef USE_ZLIB
#include <zlib.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


#ifndef TYPE_BOOL

#define TYPE_BOOL
#define TRUE 1
#define FALSE 0
#ifndef __cplusplus
typedef unsigned int bool;
#endif

#endif

extern int robotType;

#define XYPLANE 0
#define XZPLANE 1
#define YZPLANE 2

#define MAX_STRING_LENGTH 256

/* #define MAXIMUM_PROBABILITY ((probability) 0.9999) */
/* #define MINIMUM_PROBABILITY ((probability) 1.0e-30) */
/* #define MINIMUM_UPDATE_PROBABILITY_QUOTA 1e-2 */

#define MAXIMUM_PROBABILITY ((probability) 0.95)
#define MINIMUM_PROBABILITY ((probability) 1.0e-30)
#define MINIMUM_UPDATE_PROBABILITY_QUOTA 1e-10

#define UNEXPLORED 1e-30
#define IMPOSSIBLE 1e-30

#define MAXIMUM_MAPPROBABILITY ((mapProbability) 0.99999)
#define MINIMUM_MAPPROBABILITY ((mapProbability) 0.00001)


#define ROB_RADIUS 26.7
#define ROB_HEIGHT 120.0

#define MAX_NUMBER_OF_SONARS 24
#define MAX_NUMBER_OF_LASERS 2
#define NUMBER_OF_SIMULATOR_OBJECTS 10000

#define DEFAULT_PARAMETER_FILE "localize.ini"
#define DEFAULT_LOG_FILE "localize.log"
#define ONLINE_POSITION_MARKER "online"
#define MAX_NUMBER_OF_CELLS 255

#ifndef MAXFLOAT
#define   MAXFLOAT        ((float)3.40282346638528860e+38)
#endif

typedef float mapProbability;
typedef float probability;
typedef float distance;



typedef struct{
  float x;
  float y;
} point,position;

typedef struct{
  point from;
  point to;
} lineSegment;

typedef struct{
  point center;
  float radius;
} circle;


typedef struct {
    float x;
    float y;
    float rot;
} realPosition;


typedef struct {
    int x;
    int y;
    int rot;
} gridPosition;


typedef struct{
  int type;
  float posX;
  float posY;
  float posZ;
  float width;
  float depth;
  float height;
  float rot;
} simulatorObject;



/* This struct contains the simulatorMap */
typedef struct{
  bool initialized;
  float sizeX;
  float sizeY;
  float sizeZ;
  int numberOfObjects;
  float fromX;
  float fromY;
  float toX;
  float toY;
  simulatorObject object[NUMBER_OF_SIMULATOR_OBJECTS];
  char fileName[MAX_STRING_LENGTH];
} simulatorMap;


/* This struct contains the given position / occupancy probabilities.
 */
typedef struct {
  bool initialized;
  int sizeX;
  int sizeY;
  int origsizeX;
  int origsizeY;
  int shiftedX;
  int shiftedY;
  int resolution;
  float offsetX;
  float offsetY;
  float maxRealX;
  float maxRealY;
  mapProbability average;
  mapProbability unknown;
  mapProbability** prob;
  position* freeSpace;
  int numberOfFreeCells;
} probabilityGrid;

  
/* This struct contains the image map
 */
typedef unsigned char pixel;
  
typedef struct {
  bool initialized;
  unsigned int sizeX;
  unsigned int sizeY;
  unsigned int origsizeX;
  unsigned int origsizeY;
  int resolution;
  float offsetX;
  float offsetY;
  float maxRealX;
  float maxRealY;
  pixel** pix;
} visionMap;

  
/* This struct contains the position (incl. rotation) probabilities. */
typedef struct {
  bool initialized;
  int sizeX;
  int sizeY;
  int sizeZ;
  int origsizeX;
  int origsizeY;
  int positionResolution;
  float offsetX;
  float offsetY;
  float maxRealX;
  float maxRealY;
  float angleResolution;
  probability minimumProbability;
  double normalizeFactor;
  probability*** prob;

  /* The next elements are necessary to be able update only planes
   * which contain high enough probabilities.
   */
  float          quotaOfValuesToBeUpdated;
  float          quotaOfPlanesToBeUpdated;
  bool*          updatePlane;
  probability*   sumOfPlane;
  probability*   maxProbabilityOfPlane;
  probability*   normalizeFactorOfPlane;
  realPosition*  summedMovementOfPlane;

} positionProbabilityGrid;

typedef struct{
  realPosition pos;
  probability prob;
  probability stdDev;
} realCell;

typedef struct{
  realCell cell[MAX_NUMBER_OF_CELLS];
  bool inMap[MAX_NUMBER_OF_CELLS];
  int numberOfCells;
  int numberOfCellsInMap;
  probability originalSumOfProbs;
} realCellList;


typedef struct{
  gridPosition pos;
  probability prob;
} gridCell;

typedef struct{
  gridCell cell[MAX_NUMBER_OF_CELLS];
  bool inMap[MAX_NUMBER_OF_CELLS];
  int numberOfCells;
  int numberOfCellsInMap;
  probability originalSumOfProbs;
} gridCellList;


/* This struct contains the information about the robot's sensing_MOVEMENTs.
 * <rotation> is mathematically correct.
 * If <rotation> is zero and the robot moves forward the robot moves
 * on the x-axis.
 * If <rotation> is zero and the robot moves sideward the robot moves
 * on the y-axis.
 */
typedef struct {
  float forward;
  float sideward;
  float rotation;
  bool isNew;
  float elapsedTime;
} movement;

typedef struct {
  float distance;
  float angle;
  float rotation;
} polarMovement;

  
/*--------------------------------------------------------------------------
 * Structs for proximity sensors.
 *------------------------------------------------------------------------*/

/* This struct contains a single distance information. */
typedef struct {
  distance dist;
  float rot;
} distanceReading;

/* This struct contains a collection of distance readings. */
typedef struct {
  bool             isNew;
  distance         maxDistance;
  int              numberOfReadings;
  distanceReading* reading;
  float            elapsedTime;
  movement         offset;
  movement*        readingOffset;
} distanceScan;

typedef struct {
  realPosition pos;
  float certainty;
  int numberOfAlignedReadings;
  int numberOfMaxRangeAllowed;
  bool isNew;
  float elapsedTime;
} wall;


/* This tructure contains the answer to a specific question. */
 typedef struct {
  bool isNew;
  int answerType;
} answer;

typedef struct {
  bool isNew;
  pixel **pix;
  unsigned int sizeX;
  unsigned int sizeY;
} cameraImage;


/*--------------------------------------------------------------------------
 *
 *------------------------------------------------------------------------*/
typedef struct {
  int radius;
  realPosition pos;
} robot;



/*--------------------------------------------------------------------------
 * Condensation.
 *------------------------------------------------------------------------*/

typedef struct {
  realPosition pos;
  double weight;
} sampleType;

  
typedef struct {
  /* Number of samples to sample from. */
  int numberOfSamples; 

  /* Number of samples to be generated by the next sampling step (depends on stdDev). */
  int desiredNumberOfSamples;

  /* Samples. */
  sampleType* sample;

  /* Mean and standard deviation of the samples.*/
  realPosition mean;
  float stdDev;
  
  /* For internal use only. */

  /* After sampling the distribution is uniform, so no more sampling necessary. */
  bool alreadySampled;
  bool alreadyNormalized;

  /* Motion since last integration of motion. */
  realPosition summedMovement;

  /* Just for copy. */
  sampleType* tmpSample;

  /* Legal area for samples. */
  float maxX;
  float maxY;
  float minX;
  float minY;
  
  /* Number of samples allocated. */
  int allocatedSamples;

  /* Should the size of the sample set be changed online? */
  int variableSampleSize;
  int minNumberOfSamples;
  float integrateThreshold;
  float fractionOfUniformSamples;
  
  /* Needed for resampling. */
  double* accumulatedProbs;
  realPosition* tmpPositions;

  /* Indicator whether the sample set has been replace via tcx.
   * In this case we have to interrupt integration of sensor information. */
  int replacedViaTcx;

  int numberOfSet;
  struct timeval timeStamp;
  struct timeval timeOfLastShift;
} sampleSet;



  
#ifdef __cplusplus
}
#endif


#endif

