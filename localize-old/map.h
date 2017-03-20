
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/map.h,v $
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
 * $Log: map.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.18  1999/01/11 19:47:52  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.17  1998/12/16 08:51:50  wolfram
 * This version does not need the libGetDistance to compute the expected Distances
 *
 * Revision 1.16  1998/11/27 09:12:04  wolfram
 * Some changes for Frank
 *
 * Revision 1.15  1998/11/19 03:14:28  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.14  1998/08/23 00:01:02  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.13  1998/08/20 00:23:00  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.12  1998/04/19 10:40:37  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.11  1998/03/09 10:07:45  wolfram
 * slight changes
 *
 * Revision 1.10  1997/08/22 02:14:55  wolfram
 * If a beam reaches an unknown field we assume that maxRange is measured
 *
 * Revision 1.9  1997/08/16 21:52:03  wolfram
 * Grid maps can now be cad maps.  Obstacles are enlarged accordingly
 *
 * Revision 1.8  1997/03/31 21:18:15  wolfram
 * Localize now reads cylinders from the simulator map, small bug fix in graphic.c
 *
 * Revision 1.7  1997/03/13 17:36:37  fox
 * Temporary version. Don't use!
 *
 * Revision 1.6  1997/03/03 12:59:22  wolfram
 * Initial position probabilities are computed out of the simulator map if
 * the simulator map is available
 *
 * Revision 1.5  1997/02/11 11:04:10  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.4  1996/12/02 10:32:09  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/28 17:56:23  fox
 * *** empty log message ***
 *
 * Revision 1.2  1996/11/27 16:37:11  wolfram
 * added a procedure to fill the unknown fields of a map
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


#ifndef MAP_INCLUDE
#define MAP_INCLUDE

#include "sensings.h"
#include "general.h"
#include "file.h"

#define CUBE 0
#define CYLINDER 1
#define GRID_MAP_EXTENSION ".map"


/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

/**************************************************************************
 **************************************************************************
 * Structures.
 **************************************************************************
 **************************************************************************/

typedef struct {
  int cadMap;
  float unknown;
  float desiredResolution;
  char mapFileName[MAX_STRING_LENGTH];
} mapParameters;


/**************************************************************************
 **************************************************************************
 * Other functions.
 **************************************************************************
 **************************************************************************/

mapProbability
mapAverage(probabilityGrid m);

mapProbability
positionProbabilityMap(int x, int y, probabilityGrid m, float additionalSize);

mapProbability
occupancyProbabilityMapUnknown(int x, int y, probabilityGrid m);

mapProbability
occupancyProbabilityMap(int x, int y, probabilityGrid m);

mapProbability**
mapFields(int sizeX, int sizeY);

bool
readGridMap(char *mapname, char *extension, probabilityGrid *m);

bool
readVisionMap(char *mapname, char *extension, visionMap *m,
	      probabilityGrid *map);

bool
writeVisionMap(char *mapname, char *extension, visionMap *m);

bool
readSimulatorMapFile( char *fileName, simulatorMap *simMap);


bool
writeGridMap(char* mapFileName, char *extension, probabilityGrid *map);

void
computeGridMap( simulatorMap *simMap,
		int mapResolution,
		float additionalObjectSize,
		float fromZ,
		float toZ,
		probabilityGrid *m);

void
checkMapConsistency( int map1SizeX, int map1SizeY, int map1Res,
		     int map2SizeX, int map2SizeY, int map2Res,
		     char* fileName, char* mapExtension, char* extension);

int
writeMap(probabilityGrid m);

void
invertMap( probabilityGrid *map);

void
setStatistics( probabilityGrid *m);

void
normalizeMap( probabilityGrid* map);

void
normalizeMapToSum( probabilityGrid* map,
		   probability desiredSum);

int
initializeProbabilityGrid(probabilityGrid *m);

float
realCoordinateOfMapCoordinate(int x, float offset, int resolution);

float 
floatMapCoordinateOfRealCoordinate( float x, float offset, int resolution) ;

int
mapCoordinateOfRealCoordinate(float x, float offset, int resolution);

probability
probOfRealCoordinate( float x, float y, probabilityGrid* map);

int
coordinateInMap(int x, int y, probabilityGrid m);

probability
weightedOccupancyProbability( realCellList* cellList,
			      probabilityGrid* initialOccupancyProbs);

void
fillProbabilityGrid(probabilityGrid *m);

probabilityGrid
planMap( probabilityGrid* map, float additionalSize);

void
incorporatePartialMap( unsigned char* values,
		       int sizeX, int sizeY,
		       int offsetX, int offsetY,
		       float resolution,
		       probabilityGrid* map);

float
simulatorObjectDistance(simulatorMap *simMap,
			float realPosX, float realPosY,
			float sensorHeight,
			float cosRot, float sinRot, float maxRange);

probabilityGrid
invertedMap( probabilityGrid *map);

void
setFreeSpace( probabilityGrid* positionProbs);

#endif












