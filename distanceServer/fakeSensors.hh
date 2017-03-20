#include <stdio.h>
#include "store.hh"

#ifdef __cplusplus
extern "C" {
#endif
void  installSimMap(FILE *mapfd);
void  installSimMapNoBorder(FILE *mapfd);
int getDistance(float fromX, float fromY, float fromZ,
		float directionX, float directionY, float directionZ, 
		float maxDistance,
		float *distance);

  /* for backwards compatibility */
int getRayLangth(float PosX, float PosY, float PosZ,
		float EndX, float EndY, float *dist);

int rayObstacleHitPoint(float x, float y, float z,
			float dx, float dy, float dz,
			float maxLength,
			float *hx, float *hy, float *hz,
			char **obstacle_hit);

#ifdef __cplusplus
}
#endif
