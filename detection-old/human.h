#ifndef HUMAN_INCLUDE
#define HUMAN_INCLUDE

#include "general.h"

typedef struct {
  int sizeX, sizeY;
  int** beam;
  int** minIndex;
  int** maxIndex;
  int** numberOfIndices;
} beamLookupTable;

extern humanProbTable humanProbs;
extern probabilityGrid humanMap;

void
initializeHumanMapping();

void
integrateLaserScan( distanceScan* laserScan, humanProbTable* humanProbs,
		    probabilityGrid* humanMap);

#endif
