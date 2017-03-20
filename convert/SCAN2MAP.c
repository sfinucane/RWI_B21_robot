
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/SCAN2MAP.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:28 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: SCAN2MAP.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.8  1999/04/27 14:59:34  fox
 * Fixed a bug so that it compiles.
 *
 * Revision 1.7  1998/11/18 18:12:25  wolfram
 * Fixed a bug
 *
 * Revision 1.6  1998/11/16 18:48:58  wolfram
 * Minor changes
 *
 * Revision 1.5  1998/11/11 13:32:39  wolfram
 * Adopted it to the new scan-file format
 *
 * Revision 1.4  1998/03/29 00:01:12  wolfram
 * Unknown region is now computed
 *
 * Revision 1.3  1998/03/28 21:44:45  wolfram
 * Files are closed at the right point
 *
 * Revision 1.2  1998/02/21 15:18:58  wolfram
 * DISPLAYMAP now adopts the scale according to the size of the map.
 * SCAN2MAP now has several options. Scan files are
 * given as command line arguments.
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <values.h>

typedef float probability;
#define BUFSIZE 1024
#define MAXNUMBEROFBEAMS 10000000
#define MINIMUMPROBABILITY 1e-3
#define DEG_360 (M_PI + M_PI)


double 
normalizedAngle(double angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}

double
deg2Rad(double x)
{
  return x * 0.017453293;
}


double
rad2Deg(double x)
{
  return x * 57.29578;
}

typedef struct {
  int sizeX;
  int sizeY;
  int resolution;
  double **prob;
} probabilityGrid;


long
round(double x) {
  return (long) floor(x + 0.5);
}



double
fSqr( double x)
{
  return x * x;
}

double
pointDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(fSqr(x1 - x2) + fSqr(y1 - y2));
}


int
iMax( int x, int y)
{
 return ( x > y ? x : y);
}

int
iMin( int x, int y)
{
 return ( x > y ? y : x);
}



double
integratedOccupancyProbability(int x, int y, int delta, probabilityGrid *map)
{
  int i, j;
  int startI, startJ, endI, endJ;

  double prob = 0.0, dist, weightSum = 0.0, maxDist;
  
  startI = iMax(0, x - delta);
  endI = iMin(map->sizeX, x + delta + 1);

  startJ = iMax(0, y - delta);
  endJ = iMin(map->sizeY, y + delta + 1);

  maxDist = pointDistance(delta, delta, 0, 0);


  for (i = startI; i < endI; i++)
    for (j = startJ; j < endJ; j++){
      float weight;
      if (map->prob[i][j] >= 0){
	dist = pointDistance(i,j,x,y);
	weight = fabs(maxDist - dist) / maxDist;
	weightSum += weight;
	prob += weight * map->prob[i][j];
      }
    }
  if (weightSum == 0.0){
     prob = map->prob[x][y];
     fprintf(stderr, "Null: %d %d\n", x, y);
  }
  else
     prob /= weightSum;

  if (prob < MINIMUMPROBABILITY)
    prob = MINIMUMPROBABILITY;
  else if (prob > 1-MINIMUMPROBABILITY)
    prob = 1-MINIMUMPROBABILITY;
  
  return prob;
}




void
writeGridMap(probabilityGrid *map)
{
  int x,y;
  FILE *fp;

  
  fp = stdout;

  fprintf(fp, "robot_specifications->resolution %d\n", map->resolution);
  fprintf(fp, "robot_specifications->autoshifted_x %f\n",
	  (float) 0);
  fprintf(fp, "robot_specifications->autoshifted_y %f\n",
	  (float) 0);
  fprintf(fp, "global_map[0]: %d %d\n", map->sizeY, map->sizeX);

  for (x = 0; x < map->sizeX; x++){
    for (y = 0; y < map->sizeY; y++)
      if (map->prob[x][y] >= 0.0)
	fprintf(fp, "%.3f ", 1-map->prob[x][y]);
      else
	fprintf(fp, "-1 ");
    fprintf(fp, "\n");
  }
  
  fclose(fp);
  
}


double 
floatMapCoordinateOfRealCoordinate( double x, double offset, int resolution) {
  return ( x - offset) / resolution;
}


int
nextField(double x, double deltaX)
{
  if (deltaX > 0)
    return (int) round(x + 0.5);
  else if (deltaX == 0.0)
    return (int) round( x - 0.5);
  else
    return (int) (--x);
}


double
sqr(double x){
  return x*x;
}


void
computeGridMap( double sizeX,
		double sizeY,
		double *startX,
		double *startY,
		double *endX,
		double *endY,
		double *angle,
		int numberOfBeams,
		int mapResolution,
		probabilityGrid *m)
{
  int i, j, beam;
  int **hit, **cnt;
  double mapFloatStartX, mapFloatStartY;
  int mapEndX, mapEndY;
  
  m->sizeX = sizeX / (double) mapResolution + 1;
  m->sizeY = sizeY / (double) mapResolution + 1;
  m->resolution = mapResolution;
  

  hit = (int **) malloc(m->sizeX * sizeof(int));
  cnt = (int **) malloc(m->sizeX * sizeof(int));
  m->prob = (double **) malloc( m->sizeX * sizeof(double*));
  for (i = 0; i < m->sizeX; i++){
    hit[i] = ( int *) malloc( m->sizeY * sizeof(int));
    cnt[i] = ( int *) malloc( m->sizeY * sizeof(int));
    m->prob[i] = (double *) malloc( m->sizeY * sizeof(double));
  }    
  
  /* filling everything white */
  
  for (i = 0; i < m->sizeX; i++)
    for (j = 0; j < m->sizeY; j++){
      hit[i][j] = cnt[i][j] = 0;
      m->prob[i][j] = MINIMUMPROBABILITY;
    }

  for (beam = 0; beam < numberOfBeams; beam++){
    double x, y, nextX, nextY, stepsNextX, stepsNextY;
    double deltaX, deltaY, dist;
    dist = sqrt(sqr(startX[beam]-endX[beam])
		       + sqr(startY[beam] - endY[beam]));
    if (1 || dist < 600){
      float mapDist=0.0;
      mapFloatStartX = floatMapCoordinateOfRealCoordinate( startX[beam],
							   0,
							   m->resolution);
      mapFloatStartY = floatMapCoordinateOfRealCoordinate( startY[beam],
							   0,
							   m->resolution);
      
      i = x = mapFloatStartX;
      j = y = mapFloatStartY;
      
      mapEndX = (int) floatMapCoordinateOfRealCoordinate( endX[beam],
							  0,
							  m->resolution);
      mapEndY = (int) floatMapCoordinateOfRealCoordinate( endY[beam],
							  0,
							  m->resolution);
      if (0) fprintf(stderr, "(%d %d) -> (%d %d)\n", i, j, mapEndX, mapEndY);
      deltaX = cos(angle[beam]);
      deltaY = sin(angle[beam]);
      while (x > 0 && x != mapEndX && x < m->sizeX
	     && y > 0 && y != mapEndY && y < m->sizeY && mapDist < dist){
	cnt[(int) x][(int) y]++ ;	
	if (fabs(fabs(deltaX) - fabs(deltaY)) >= 1e-4){
	  nextX = nextField(x, deltaX);
	  nextY = nextField(y, deltaY);
	
	  if (deltaX == 0.0)
	    stepsNextX = MAXFLOAT;
	  else
	    stepsNextX = (nextX - x) / deltaX;
	
	  if (deltaY == 0.0)
	    stepsNextY = MAXFLOAT;
	  else
	    stepsNextY = (nextY - y) / deltaY;
	  
	  if (stepsNextX < stepsNextY){
	    x = nextX;
	    y += stepsNextX * deltaY;
	  }
	  else{
	    y = nextY;
	  x += stepsNextY * deltaX;
	  }
	}
	else{
	  if (deltaX > 0)
	    x++;
	  else if (deltaX < 0)
	  x--;
	  if (deltaY > 0)
	    y++;
	  else if (deltaY < 0)
	    y--;
	}
	mapDist = sqrt(sqr(x - mapFloatStartX) + sqr(y - mapFloatStartY))
	  * m->resolution;
      }
      if (x > 0 && x < m->sizeX && y > 0 && y < m->sizeY){
	cnt[(int) x][(int) y]++;
	hit[(int) x][(int) y]++;
      }
    }
  }
    
  for (i = 0; i < m->sizeX; i++){
    for (j = 0; j < m->sizeY; j++){
      if (cnt[i][j] > 0)
	m->prob[i][j] = (double) hit[i][j] / cnt[i][j];
      else
	m->prob[i][j] = -1.0;
    }
  }
}

void
processUnknown(probabilityGrid *map, double threshold, int clipRange)
{
  double **prob;
  int i,x, y, x1, y1;

  prob = (double **) malloc( map->sizeX * sizeof(double*));
  for (i = 0; i < map->sizeX; i++)
    prob[i] = (double *) malloc( map->sizeY * sizeof(double));


  while (clipRange > 0){

    /* copy map */
    for (x = 0; x < map->sizeX; x++)
      for (y = 0; y < map->sizeY; y++)
	prob[x][y] = map->prob[x][y];
    
    
    for (x = 2; x < map->sizeX - 2; x++)
      for (y = 2; y < map->sizeY - 2; y++)
	if (map->prob[x][y] < 0.0) {
	  int whiteNeighbor = 0;
	  int blackNeighbor = 0;
	  for (x1 = x-1 ; (x1 < x+2) && !whiteNeighbor; x1++){
	    for (y1 = y-1; (y1 < y+2) && !whiteNeighbor; y1++){
	      whiteNeighbor = (map->prob[x1][y1] >= 0.0
			     && map->prob[x1][y1] < 0.1);
	    }
	  }

	  for (x1 = x-2 ; (x1 < x+3) && !blackNeighbor; x1++){
	    for (y1 = y-2; (y1 < y+3) && !blackNeighbor; y1++){
	      blackNeighbor = (map->prob[x1][y1] >= 0.3);
	    }
	  }
	  
	  if (whiteNeighbor && !blackNeighbor)
	    prob[x][y] = MINIMUMPROBABILITY;
	}

    /* copy map back */
    for (x = 0; x < map->sizeX; x++)
      for (y = 0; y < map->sizeY; y++)
	map->prob[x][y] = prob[x][y];

    clipRange--;
  }
  
  for (i = 0; i < map->sizeX; i++)
    free(prob[i]);
  free(prob);

}



void
clipMap(probabilityGrid *map, double threshold, int clipRange)
{
  
  int x, y, x1, y1;

  for (x = clipRange; x < map->sizeX-clipRange -1; x++)
    for (y = clipRange; y < map->sizeY-clipRange-1; y++)
      if (map->prob[x][y] > 0 && map->prob[x][y] < threshold){
	int empty;
	empty = 1;

	for (x1 = x-clipRange ; x1 < x+clipRange + 1 && empty; x1++){
	  for (y1 = y-clipRange; y1 < y+clipRange + 1 && empty; y1++){
	    empty = map->prob[x1][y1] < threshold;
	  }
	}
	if (empty)
	  map->prob[x][y] = MINIMUMPROBABILITY;
      }
}

#define ROT_TOKEN "-rot"
#define THRESH_TOKEN "-threshold"
#define CLIP_TOKEN "-cliprange"
#define RESOLUTION_TOKEN "-resolution"
#define HELP_TOKEN "-help"

void
main( int argc, char *argv[] ){

  char fileName[BUFSIZE], line[BUFSIZE];
  double minX, maxX, minY, maxY;
  double *angle, *posX, *posY, *endX, *endY;
  double rotAngle = 0.0;
  long int beam, numberOfBeams;
  probabilityGrid map;
  int i;
  int cnt = 0;
  double threshold = 0.0;
  int range = 0;
  int resolution = 10;
  int fileIndex = 1;
  
  FILE *fp = NULL;

  fprintf(stderr, "argc: %d\n", argc);
  
  sprintf(line, "usage: %s [%s angle] [%s threshold] [%s cliprange] [%s resolution] [%s] file1 [file2 ...]\n",
	  argv[0],
	  ROT_TOKEN, THRESH_TOKEN, CLIP_TOKEN,
	  RESOLUTION_TOKEN,
	  HELP_TOKEN);
	  
  if (argc == 1){
    fprintf(stderr, "%s\n", line);
    exit(0);
  }

  for (i = 1; i < argc; i++){
    if ((strcmp(argv[i],ROT_TOKEN)==0)) {
      if ( i < argc - 1){
	rotAngle = atof(argv[++i]);
	fileIndex = i+1;
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword %s.\n", ROT_TOKEN);
	exit;
      }
    }
    else if ((strcmp(argv[i],RESOLUTION_TOKEN)==0)) {
      if ( i < argc - 1){
	resolution = atoi(argv[++i]);
	fileIndex = i+1;
      }
      else {
	fprintf( stderr, "ERROR: int must follow keyword %s.\n",
		 RESOLUTION_TOKEN);
	exit;
      }
    }
    else if ((strcmp(argv[i], THRESH_TOKEN)==0)) {
      if ( i < argc - 1){
	threshold = atof(argv[++i]);
	fileIndex = i+1;
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword %s.\n",
		 THRESH_TOKEN);
	exit;
      }
    }
    else if ((strcmp(argv[i],CLIP_TOKEN)==0)) {
      if ( i < argc - 1){
	range = atoi(argv[++i]);
	fileIndex = i+1;
      }
      else {
	fprintf( stderr, "ERROR: int must follow keyword %s. CLIP_TOKEN\n");
	exit;
      }
    }
    else if (strcmp(argv[i], HELP_TOKEN) == 0){
      fprintf(stderr, "%s", line);
      exit(0);
    }
  }
    
  posX = (double *) malloc( MAXNUMBEROFBEAMS * sizeof(double));
  posY = (double *) malloc( MAXNUMBEROFBEAMS * sizeof(double));
  endX = (double *) malloc( MAXNUMBEROFBEAMS * sizeof(double));
  endY = (double *) malloc( MAXNUMBEROFBEAMS * sizeof(double));
  angle = (double *) malloc( MAXNUMBEROFBEAMS * sizeof(double));
  numberOfBeams = 0;
  
  
  while ( fileIndex < argc) {
    float xPos, yPos;
    float rot = 0;
    fprintf(stderr, "scanning file %s\r", argv[fileIndex]);
    fileIndex++;
    if ((fp = fopen(argv[fileIndex], "r")) != NULL){
      while (!feof(fp) && fgets(line, BUFSIZE, fp) &&
	     strncmp(line, "2D-Scan", 7)!= 0);

      while (!feof(fp) && fgets(line, BUFSIZE, fp) &&
	     strncmp(line, "RobotPos:", 9)!= 0);
      if (sscanf(&line[9], "%f %f %f",
		 &xPos,
		 &yPos,
		 &rot) != 3){
	fprintf(stderr, "ERROR: could not scan position line %s\n", line);
	exit(0);
      }
	
      while (!feof(fp) && fgets(line, BUFSIZE, fp) &&
	     strncmp(line, "DATA", 4)!= 0);
      
      while (!feof(fp) && numberOfBeams < MAXNUMBEROFBEAMS ){
	int dist;
	float beamRot; 
	fgets(line, BUFSIZE, fp);
	if (sscanf(line, "%f %d", &beamRot, &dist) == 2) {
	  posX[numberOfBeams] = xPos * 0.1;
	  posY[numberOfBeams] = yPos * 0.1;
	  
	  angle[numberOfBeams] = rot + beamRot;
	  endX[numberOfBeams] = posX[numberOfBeams]
	  + dist*0.1 * cos(angle[numberOfBeams]);
	  endY[numberOfBeams] = posY[numberOfBeams]
	    + dist*0.1 * sin(angle[numberOfBeams]);
	  numberOfBeams++;
	}
      }
      cnt++;
      fclose(fp);
    }
  }
  /* rotate map */
  if (rotAngle != (double) 0) {
    for (beam = 0; beam < numberOfBeams; beam++){
      double alpha,dist;
      alpha = atan2( posY[beam], posX[beam]);
      dist = sqrt(fSqr(posX[beam]) + fSqr(posY[beam]));
      posX[beam] = dist * cos(alpha + deg2Rad(rotAngle));
      posY[beam] = dist * sin(alpha + deg2Rad(rotAngle));
      angle[beam] += deg2Rad(rotAngle);
      alpha = atan2(endY[beam], endX[beam]);
      dist = sqrt(fSqr(endX[beam]) + fSqr(endY[beam]));
      endX[beam] = dist * cos(alpha + deg2Rad(rotAngle));
	endY[beam] = dist * sin(alpha + deg2Rad(rotAngle));
    }
  }
  

  

  
  minX = minY = 1e40;
  maxX = maxY = -1e40;
  for (beam = 0; beam < numberOfBeams; beam++){
    if (endX[beam] < minX)
      minX = endX[beam];
    if (endX[beam] > maxX)
      maxX = endX[beam];
    if (endY[beam] < minY)
      minY = endY[beam];
    if (endY[beam] > maxY)
      maxY = endY[beam];
    if (posX[beam] < minX)
      minX = posX[beam];
    if (posX[beam] > maxX)
      maxX = posX[beam];
    if (posY[beam] < minY)
      minY = posY[beam];
    if (posY[beam] > maxY)
      maxY = posY[beam];
  }

  for (beam = 0; beam < numberOfBeams; beam++){
    posX[beam] -= minX;
    endX[beam] -= minX;
    posY[beam] -= minY;
    endY[beam] -= minY;
  }



  
  maxX -= minX;
  maxY -= minY;
     
  computeGridMap( maxX, maxY, posX, posY, endX, endY, angle,
		  numberOfBeams, resolution, &map);
  if (threshold > 0.0)
    clipMap(&map, threshold, range);
  if (0) processUnknown(&map, threshold, 10);
  if (1) writeGridMap(&map);

  if (0) printf("# %f %f %f %f\n", minX, maxX, minY, maxY);
  if (0) for (beam = 0; beam < numberOfBeams; beam++)
    printf("%f %f\n", endX[beam], endY[beam]);


  
  
  
  fprintf(stderr, "\n done\n");
  exit(1);
}

















