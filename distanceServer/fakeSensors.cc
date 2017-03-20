
#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <stdio.h>
#include <stdlib.h>

int keep_gcc272_happy_0;

#include "surface.hh"
#include "store.hh"
#include "mapreader.hh"

#include "fakeSensors.hh"

float sonar_range = 500.0;

t_ObstacleGrid *obstore;

extern t_surface *default_surface;
// the default_surface must not be referenced in this application !!!

void
installSimMap(FILE *mapfd)
{
  float x1, y1, x2, y2;
  float xin, yin, ori_in;
  t_mapReader sim_map(mapfd);
  if(!sim_map.getMapDimensions(x1, y1, x2, y2)) {
    fprintf(stderr, "Simulator: Mapfile corrupted!\n");
    exit(0);
  }
  if(!sim_map.getRobotPosition(xin,yin,ori_in)) {
    fprintf(stderr, "Simulator: Mapfile corrupted!\n");
    exit(0);
  }
  obstore = new t_ObstacleGrid(x1, y1, x2, y2);  
  obstore->read_obstacles(sim_map);
}

void
installSimMapNoBorder(FILE *mapfd)
{
  float x1, y1, x2, y2;
  float xin, yin, ori_in;
  t_mapReader sim_map(mapfd);
  if(!sim_map.getMapDimensions(x1, y1, x2, y2)) {
    fprintf(stderr, "Simulator: Mapfile corrupted!\n");
    exit(0);
  }
  if(!sim_map.getRobotPosition(xin,yin,ori_in)) {
    fprintf(stderr, "Simulator: Mapfile corrupted!\n");
    exit(0);
  }
  obstore = new t_ObstacleGrid(x1, y1, x2, y2, 0);  
  obstore->read_obstacles(sim_map);
}

int
getDistance(float PosX, float PosY, float PosZ,
	    float DirX, float DirY, float DirZ,
	    float maxLength,
	    float *distance)
{
  float angle;
  t_surface *surface;
  if(obstore->get_distance(t_Vector3f(PosX, PosY, PosZ),
			   t_Vector3f(DirX, DirY, DirZ),
			   maxLength,
			   distance)) {
    return 1;
  }
  else
    return 0;
}

/* for backwards compatibility */
int getRayLength(float PosX, float PosY, float PosZ,
		float EndX, float EndY, float *dist)
{
  float dx = EndX - PosX;
  float dy = EndY - PosY;

  float angle = atan2(dy,dx);
  float length = sqrt(dx*dx+dy*dy);
  if(obstore->get_distance(t_Vector3f(PosX, PosY, PosZ),
			   t_Vector3f(cos(angle), sin(angle), 0.0),
			   length,
			   dist)) {
    return 1;
  }
  else
    return 0;  
}

int
rayObstacleHitPoint(float x, float y, float z,
		    float dx, float dy, float dz,
		    float maxLength,
		    float *hx, float *hy, float *hz,
		    char **obstacle_hit)
{

  t_Vector3f pointHit;
  int hit;
  t_Obstacle* ohit;
  hit = obstore->rayObstacleIntersection(t_Vector3f(x,y,z),
				   t_Vector3f(dx,dy,dz),
				   maxLength,
				   pointHit,
				   &ohit);
  *hx = pointHit.x();
  *hy = pointHit.y();
  *hz = pointHit.z();
  if(hit) *obstacle_hit = ohit->label();
  else *obstacle_hit = NULL;
  return hit;
}
