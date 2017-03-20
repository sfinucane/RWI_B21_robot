#ifndef SCANALIGNMENT_INCLUDE
#define SCANALIGNMENT_INCLUDE

#include "general.h"
#include "proximity.h"
#include "map.h"


realPosition
scanMatchingPosition(realPosition pos, simulatorMap *simMap,
		     actionInformation *info);


void
alignInit(int resolution, float maxTransError, float maxRotError,
	  simulatorMap *simMap);

void
setRobot(float x, float y, float rot);

void
getRobot(float *x, float *y, float *rot);

void
laserInitRHINO();

int
correctRobotPosition(float *reading);

#endif
