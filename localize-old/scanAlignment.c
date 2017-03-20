#include <math.h>


#include "map.h"
#include "general.h"
#include "proximity.h"
#include "scanAlignment.h"
#include "allocate.h"
#include "sensings.h"

#ifndef MAXFLOAT
#define MAXFLOAT FLT_MAX
#endif

#define EXCLUDED_READING (-laserSpec.maxRange)

static simulatorMap *globalSimMap;


typedef struct {
  float x;
  float y;
} t_Vector2f;

typedef struct {
  int resolution;
  float maxOffset;
  float maxRotError;
  float residualX;
  float residualY;
  float residualOri;
} t_AlignSpec;

typedef struct {
  float f_startAngle;
  float f_angleResolution;
  float f_numberOfReadings;
  t_Vector2f f_offset;
  float r_startAngle;
  float r_angleResolution;
  float r_numberOfReadings;
  t_Vector2f r_offset;
  float maxRange;
} t_LaserSpec;

typedef struct {
  t_Vector2f pos;
  float ori;
} t_RobotState;

static t_LaserSpec laserSpec;
static t_AlignSpec alignSpec;
static t_RobotState robotState;

void
setScans(actionInformation *info,
	 float* scan){
  int i;
  sensing_PROXIMITY frontLaser = info->actualSensings.frontLaser;
  sensing_PROXIMITY rearLaser = info->actualSensings.rearLaser;
  
  for (i = 0; i < frontLaser.numberOfReadings; i++){
    scan[i] = frontLaser.reading[i].dist;
    if (0) writeLog( "%d %f #m\n", i, scan[i]);
  }
  for (i = 0; i < rearLaser.numberOfReadings; i++){
    scan[i+frontLaser.numberOfReadings] = rearLaser.reading[i].dist;
    if (0) writeLog( "%d %f #m\n", i+frontLaser.numberOfReadings,
	      scan[i+frontLaser.numberOfReadings]);
  }
}

realPosition
scanMatchingPosition(realPosition pos, simulatorMap *simMap,
		     actionInformation *info)
{
  realPosition newPos;
  static float* scan = NULL;
  static bool firstTime = TRUE;
  if (firstTime){
    laserInitRHINO();
    alignInit(1, 40, 10, simMap);
    scan = allocate1D(360, FLOAT);
  }
  setRobot(pos.x, pos.y, pos.rot);
  setScans(info, scan);
  fprintf(stderr, "# OLD: %f %f %f\n", pos.x, pos.y, pos.rot);
  correctRobotPosition(scan);
  getRobot(&newPos.x, &newPos.y, &newPos.rot);
  fprintf(stderr, "# NEW: %f %f %f\n", newPos.x, newPos.y, newPos.rot);
  return newPos;
}


static t_Vector2f
scalarMult(float a, t_Vector2f v)
{
  t_Vector2f p;
  p.x = a * v.x;
  p.y = a * v.y;
  return p;
}

static t_Vector2f
vecSub(t_Vector2f v1, t_Vector2f v2)
{
  t_Vector2f p;
  p.x = v1.x - v2.x;
  p.y = v1.y - v2.y;
  return p;
}

static t_Vector2f
vecAdd(t_Vector2f v1, t_Vector2f v2)
{
  t_Vector2f p;
  p.x = v1.x + v2.x;
  p.y = v1.y + v2.y;
  return p;
}

static t_Vector2f
vecRot(float alpha, t_Vector2f v)
{
  t_Vector2f p;
  p.x = v.x*cos(alpha)-v.y*sin(alpha);
  p.y = v.x*sin(alpha)+v.y*cos(alpha);
  return p;
}

static float 
scalarProd(t_Vector2f v1, t_Vector2f v2)
{
  return v1.x*v2.x + v1.y*v2.y;
}

static float 
beam_angle(int index)
{
  float angle;
  if(index < laserSpec.f_numberOfReadings) {
    angle = robotState.ori + laserSpec.f_startAngle + 
      index * laserSpec.f_angleResolution;
  }
  else {
    angle = robotState.ori + laserSpec.r_startAngle +
      (index-laserSpec.f_numberOfReadings) * 
      laserSpec.f_angleResolution;
  }
  return angle;
}

static t_Vector2f 
beam_direction(int index)
{
  float alpha = beam_angle(index);
  t_Vector2f p;
  p.x = cos(alpha);
  p.y = sin(alpha);
  return p;
}


static t_Vector2f
beam_start(int index)
{
  t_Vector2f offset;
  if(index < laserSpec.f_numberOfReadings) {
    offset = laserSpec.f_offset;
  }
  else {
    offset = laserSpec.r_offset;
  }
  return vecAdd(robotState.pos, vecRot(robotState.ori, offset));
}

static float
dist(int index)
{
  t_Vector2f dir;
  t_Vector2f start;
  dir = beam_direction(index);
  start = beam_start(index);
  return simulatorObjectDistance(globalSimMap, start.x, start.y, 40.0,
				 dir.x, dir.y, laserSpec.maxRange);
}


static float cachedDistance[360];
static void
cacheDistances()
{
  int i;
  for(i = 0; i < laserSpec.f_numberOfReadings+laserSpec.r_numberOfReadings;
      i++){
    cachedDistance[i] = dist(i);
    if (0) writeLog( "%d %f #e\n", i, cachedDistance[i]);
  }
}

/*****************************************************************************/
/*                                                                           */
/* Method: estimateScanOffset                                                */
/*                                                                           */
/* Computes the least squares  fit solution for the (x,y,omega) offset       */
/* between two laser scans                                                   */
/*                                                                           */
/* taken from Feng Lu, Evangelios Milios                                     */
/* Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans  */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

static int
estimateScanOffset(int len, 
		   float *reading1, 
		   float *reading2,
		   float* r2_angles,
		   float *omega, float *Tx, float *Ty)
{
  int i, n;
  float alpha;
  float rW, rM, rM_a;
  float Sxxp = 0;
  float Syyp = 0;
  float Sxyp = 0;
  float Syxp = 0;
  float xm = 0, ym = 0, xpm = 0, ypm = 0;

  *Tx = 0; *Ty = 0; *omega = 0;
  n = 0;
  for(i = 0; i < len; i += alignSpec.resolution) {
    rW = reading1[i];
    rM = reading2[i];
    rM_a = r2_angles[i];

    if( (rM < 0 || rW < 0) || rW >= laserSpec.maxRange) continue;

    alpha = beam_angle(i);
    if( rW < laserSpec.maxRange && rM < laserSpec.maxRange) {
      xm += rW * cos(alpha);
      ym += rW * sin(alpha);
      xpm += rM * cos(rM_a);
      ypm += rM * sin(rM_a);
      n++;
    }
  }

  if(n  == 0) return n;

  xm /= n;
  ym /= n;
  xpm /= n;
  ypm /= n;
  
  for(i=0; i<len; i+= alignSpec.resolution) {
    rW = reading1[i];
    rM = reading2[i];
    rM_a = r2_angles[i];
    if( (rM < 0 || rW < 0) || rW >= laserSpec.maxRange) continue;

    if( rW < laserSpec.maxRange && rM < laserSpec.maxRange) {
      alpha = beam_angle(i);
      Sxxp += (rW * cos(alpha) - xm) * ( rM * cos(rM_a) - xpm);
      Sxxp += (rW * sin(alpha) - ym) * ( rM * sin(rM_a) - ypm);
      Sxyp += (rW * cos(alpha) - xm) * ( rM * sin(rM_a) - ypm);
      Syxp += (rW * sin(alpha) - ym) * ( rM * cos(rM_a) - xpm);
    }
  }
  
  *omega = atan( (Sxyp-Syxp)/(Sxxp-Syyp));
  *Tx = xpm - (xm*cos(*omega)-ym*sin(*omega));
  *Ty = ypm - (xm*sin(*omega)+ym*cos(*omega));
  return n;
}

static void
closest_point_vscan(float delta,
		    int scan_size,
		    float* reading,
		    float* v_reading,
		    float* v_angles)
{
  int i, j, angle = 0;
  int beam_valid = 0;
  float rW, rM, rM_min = 0, d, dmin;
  t_Vector2f pW, pMW;
  for(i = 0; i < scan_size; i += alignSpec.resolution) {
    rW = reading[i];
    beam_valid = (rW >= 0 && rW < laserSpec.maxRange);
    
    if(beam_valid) {
      dmin = MAXFLOAT;
      for(j = - ((int)(delta+0.5)); j < ((int)(delta+0.5))-1; j++) {
	d = 0;
	rM = cachedDistance[(360+i+j) % 360];
	if(rM >= 0) {
	  pW = scalarMult(rW, beam_direction(i+j));
	  pMW = vecSub(scalarMult(rM, beam_direction(i)), pW);
	  d = scalarProd(pMW,pMW);
	  if(d < dmin) {
	    rM_min = cachedDistance[(360+i+j) % 360];
	    angle = i+j;
	    dmin=d;
	  }
	}
      }
    }      
    if(beam_valid && sqrt(dmin) < alignSpec.maxOffset) {
      v_reading[i] = rM_min;
      v_angles[i] = beam_angle(angle);
    }
    else {
      v_reading[i] = EXCLUDED_READING;
    }
  }
}
  
static void
matching_range_vscan(float delta,
		     int scan_size,
		     float* reading,
		     float* v_reading,
		     float* v_angles)
{
  int i,j, angle = 0, n;
  float rW, rM, d, dmin;
  dmin = MAXFLOAT;
  for(j = - ((int)(delta+0.5)); j < ((int)(delta+0.5))-1; j++) {
    d = 0;
    n = 0;
    for(i = 0; i < scan_size; i += alignSpec.resolution) {
      rW = reading[i];
      if(rW >= 0 && rW < laserSpec.maxRange) {
	rM = cachedDistance[(360+i+j) % 360];
	if(rM >= 0) {
	  d += fabs(rW-rM);
	}
	n++;
      }
    }
    if(d/n < dmin) {
      dmin = d/n;
      angle = j;
    }
      
  }
  for(i = 0; i < scan_size; i += alignSpec.resolution) {
    v_reading[i] = cachedDistance[(360+i+angle) % 360];
    v_angles[i] = beam_angle(i+angle);
  }
}

static float
diffSqr(float a, float b)
{
  float f = a-b;
  return f*f;
}

static
float
matchError(int scan_size, float *reading)
{
  int i, n = 0;
  float inc;
  float error = 0;
  for (i = 0; i < scan_size; i += alignSpec.resolution) {
    if(reading[i] >= 0 && reading[i] < laserSpec.maxRange) {
      inc = 0.5 * diffSqr(reading[i],cachedDistance[i]);
      inc += 0.25 * diffSqr(reading[i],cachedDistance[(360+i-1)%360]);
      inc += 0.25 * diffSqr(reading[i],cachedDistance[(i+1)%360]);
      if(inc < 1600){
	error += inc;
	n++;
      }
    }
  }
  return error / n;
}

int
correctRobotPosition(float *reading)
{
  float v_reading[360];
  float v_angles[360];
  float omega;
  float Tx,Ty;

  int iter;

  float allowed_rot = alignSpec.maxRotError;
  float dummy, error;
  int scan_size = laserSpec.f_numberOfReadings +
    laserSpec.r_numberOfReadings;

#ifdef DEBUG
  fprintf(stderr, "start: %f %f %f\n", 
	  robotState.pos.x, robotState.pos.y, robotState.ori);
#endif

  iter=0;
  cacheDistances();
  
  /*
  clearScanWindow();
  plotScan(2, 360, robotState.pos.x, robotState.pos.y, robotState.ori,
	   reading);
    plotScan(3, 360, robotState.pos.x, robotState.pos.y, robotState.ori,
     cachedDistance);
  */
  do {
    
    matching_range_vscan(allowed_rot, scan_size, reading, 
			 v_reading, v_angles);
    estimateScanOffset(scan_size, reading, v_reading, v_angles,
		       &omega, &Tx, &Ty);
    closest_point_vscan(allowed_rot, scan_size, reading, 
			v_reading, v_angles);
    estimateScanOffset(scan_size, reading, v_reading, v_angles,
		       &dummy, &Tx, &Ty);
    robotState.pos.x += Tx;
    robotState.pos.y += Ty;
    robotState.ori += omega;

    cacheDistances();

    /*
    clearScanWindow();
    plotScan(2, 360, robotState.pos.x, robotState.pos.y, robotState.ori,
	     reading);
    plotScan(3, 360, robotState.pos.x, robotState.pos.y, robotState.ori,
	     cachedDistance);
    */
    iter++; 
  } while(iter < 100
	  && (fabs(Tx) > alignSpec.residualX 
	      || fabs(Ty) > alignSpec.residualY 
	      || fabs(omega) > alignSpec.residualOri));


  error = matchError(scan_size, reading);
#ifdef DEBUG
  fprintf(stderr, "corrected %f %f %f e: %f\n", 
	  robotState.pos.x, robotState.pos.y, robotState.ori, error);  
#endif
  return error;
} 

void
alignInit(int resolution, float maxTransError, float maxRotError,
	  simulatorMap *simMap)
{
  alignSpec.resolution = resolution;
  alignSpec.maxOffset = maxTransError;
  alignSpec.maxRotError = maxRotError;
  alignSpec.residualX = 0.3;
  alignSpec.residualY = 0.3;
  alignSpec.residualOri = 0.008;
  globalSimMap = simMap;
  
}

void
setRobot(float x, float y, float rot)
{
  robotState.pos.x = x;
  robotState.pos.y = y;
  robotState.ori = rot;
}

void
getRobot(float *x, float *y, float *rot)
{
  *x = robotState.pos.x;
  *y = robotState.pos.y;
  *rot = robotState.ori;
}

void
laserInitRHINO()
{
  laserSpec.f_startAngle = 3*(M_PI/2);
  laserSpec.f_angleResolution = M_PI/180.0;
  laserSpec.f_numberOfReadings = 180;
  laserSpec.f_offset.x = 11.5;
  laserSpec.r_startAngle = 1*(M_PI/2);
  laserSpec.r_angleResolution = M_PI/180.0;
  laserSpec.r_numberOfReadings = 180;
  laserSpec.r_offset.x = -11.5;
  laserSpec.maxRange = 500;
}
