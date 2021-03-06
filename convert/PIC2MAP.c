#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <values.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/shape.h>
#include <Imlib.h>

#include "tcx.h"
#include "../localize/general.h"

#define MINIMUM_MAP_SIZE  300
#define MAP_SIZE_STEP     50
#define NUMBER_OF_CORNERS 4
#define BUFFLEN           80
#define DEFAULT_MAP_RES   15

/* These are the values sent by the base after a reset. */
#define ROBOT_INIT_POS_X 1213.3
#define ROBOT_INIT_POS_Y 1213.3
#define ROBOT_INIT_ROT   0.0

float
fAbs( float x)
{
  return (x >= 0.0 ? x : -x);
}

int
iAbs( int x)
{
  return (x >= 0.0 ? x : -x);
}

float
fMax( float x, float y)
{
 return ( x > y ? x : y);
}

int
iMax( int x, int y)
{
 return ( x > y ? x : y);
}


float
fMin( float x, float y)
{
 return ( x > y ? y : x);
}

int
iMin( int x, int y)
{
 return ( x > y ? y : x);
}


float
fSqr( float x)
{
  return x * x;
}

int
iSqr( int x)
{
  return x * x;
}

static float
pointDistance(float x1, float y1, float x2, float y2)
{
  return sqrt(fSqr(x1 - x2) + fSqr(y1 - y2));
}

float
integratedOccupancyProbability(int x, int y, int delta, probabilityGrid *map)
{
  register int i, j;
  int startI, startJ, endI, endJ;

  float prob = 0.0, dist, weightSum = 0.0;
  
  startI = iMax(0, x - delta);
  endI = iMin(map->sizeX, x + delta + 1);

  startJ = iMax(0, y - delta);
  endJ = iMin(map->sizeY, y + delta + 1);


  for (i = startI; i < endI; i++)
    for (j = startJ; j < endJ; j++){
      if (map->prob[i][j] >= 0){
	weightSum += (dist = pointDistance(i,j,x,y));
	prob += dist * map->prob[i][j];
      }
    }
  if (weightSum == 0.0){
     prob = map->prob[x][y];
     fprintf(stderr, "Null: %d %d\n", x, y);
  }
  else
     prob /= weightSum;

  if (prob != -1) {
      if (prob < MINIMUM_PROBABILITY)
	  prob = MINIMUM_PROBABILITY;
      else if (prob > MAXIMUM_PROBABILITY)
	  prob = MAXIMUM_PROBABILITY;
  }
  
  return prob;
}

int
convert_pic2map( ImlibImage *im, probabilityGrid *map )
{
  int x,y;
  float r,g,b;
  float grey;

  map->sizeX = im->rgb_width;
  map->sizeY = im->rgb_height;

  map->prob = (float **)
    malloc(map->sizeX * sizeof(float *));
  for (x = 0; x < map->sizeX; x++)
    map->prob[x] = (float *) malloc(map->sizeY * sizeof(float));
  for (x = 0; x < map->sizeX; x++)
    for (y = 0; y < map->sizeY; y++) {
      r = (float) im->rgb_data[((map->sizeY-y-1)*map->sizeX*3)+(x*3)+0];
      g = (float) im->rgb_data[((map->sizeY-y-1)*map->sizeX*3)+(x*3)+1];
      b = (float) im->rgb_data[((map->sizeY-y-1)*map->sizeX*3)+(x*3)+2];

      /* This works much better -BR */
      if ((r == 78) && (g == 170) && (b == 68)) {
        map->prob[x][y] = -1;
      }
      else {
        grey = (r+g+b)/(255*3.0);
        map->prob[x][y] = grey;
      }
    }
  return(1);
}

int
convert_pic2map2( ImlibImage *im, probabilityGrid *map )
{
  int x,y;
  probabilityGrid *tmpMap;

  tmpMap->sizeX = im->rgb_width;
  tmpMap->sizeY = im->rgb_height;

  tmpMap->prob = (float **)
    malloc(tmpMap->sizeX * sizeof(float *));
  for (x = 0; x < tmpMap->sizeX; x++)
    tmpMap->prob[x] = (float *) malloc(tmpMap->sizeY * sizeof(float));
  for (x = 0; x < tmpMap->sizeX; x++)
    for (y = 0; y < tmpMap->sizeY; y++) {
      tmpMap->prob[x][y] = (float) im->rgb_data[(y*tmpMap->sizeX*3)+x+0];
    }
  return(1);
}

void
printUnknown( FILE *fp, int n)
{
   while (n-- > 0)
      fprintf( fp, "-1 ");
}

void
usage( char *pname )
{
  fprintf(stderr,
	  "usage: %s [-resolution RES] <infile> <outfile>\n",
	  pname);
  exit(0);
}

int
main( int argc, char *argv[] )
{
  FILE *ifp, *ofp;
  
  Display *disp;
  ImlibData *id;
  XSetWindowAttributes attr;
  Window win;
  ImlibImage *im;
  Pixmap p,m;
  int w,h;

  int globalSizeX, globalSizeY;
  int extendedSizeX, extendedSizeY; 
  int top, bottom, left, right;
  int x, y;

  int argCount;
  int mapResolution = DEFAULT_MAP_RES;
  float fromHeight, toHeight;
  float additionalObjectBorder;
  probabilityGrid map;
  realPosition robotPos;

  char* inFileName;
  char* outFileName;
   
  argCount = 0;
  if (argc < 3) {
    usage(argv[0]);
  } else {
    while( argCount<argc-3 ) {
      argCount++;
      if (!strcmp(argv[argCount],"-resolution")) {
	mapResolution = atoi(argv[++argCount]);
	if (mapResolution <= 0) {
	  fprintf(stderr, "Wrong resolution: %s\n", argv[argCount]);
	  exit(0);
	} else
	  fprintf(stderr, "Map resolution: %d\n", mapResolution);
      } else {
	usage(argv[0]);
      }
    }
    argCount++;
    inFileName = argv[argCount];
    if ((ifp = fopen(inFileName,"rt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open picture file '%s'!\n",
	      inFileName);
      exit(0);
    }
    
    argCount++;
    outFileName = argv[argCount];
    if ((ofp = fopen(outFileName,"wt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open map file '%s'!\n",outFileName);
    exit(0);
    }
    
  }

  robotPos.x = robotPos.y = robotPos.rot = MAXFLOAT;

  fprintf(stderr,"open display...");
  disp=XOpenDisplay(NULL);
  fprintf(stderr,"ok\n");

  fprintf(stderr,"init imlib...");
  id=Imlib_init(disp);
  fprintf(stderr,"ok\n");

  fprintf(stderr,"read picture...");
  im=Imlib_load_image(id,inFileName);
  w=im->rgb_width;h=im->rgb_height;
  fprintf(stderr,"ok (w=%d,h=%d)\n",w,h);
  
 /* Testing -BR */
  if (!convert_pic2map(im,&map)) { 
 /* if (!convert_pic2map2(im,&map)) { */
    fprintf(stderr, "ERROR: Could not convert picture '%s'!\n",
	    inFileName);
    exit(0);

  } else {
    
    if (map.sizeX < MINIMUM_MAP_SIZE)
      extendedSizeX = MINIMUM_MAP_SIZE;
    else {
      /* extendedSizeX = ((map.sizeX / 50) + 1) * 50; */
      /* This doesn't look right -BR */
      extendedSizeX = map.sizeX;
    }
    
    if (map.sizeY < MINIMUM_MAP_SIZE)
      extendedSizeY = MINIMUM_MAP_SIZE;
    else {
     /* extendedSizeY = ((map.sizeY / 50) + 1) * 50; */
     /* likewise -BR */
      extendedSizeY = map.sizeY;
    }
    
    top = (extendedSizeY - map.sizeY) / 2;
    bottom = extendedSizeY - top - map.sizeY;
    left = (extendedSizeX - map.sizeX) / 2;
    right = extendedSizeX - left - map.sizeX;
    
    globalSizeX = extendedSizeX * mapResolution;
    globalSizeY = extendedSizeY * mapResolution;
    
    
    /* If there is a robot Position, then I can compute the correction
       parameter ! */
    if( robotPos.rot != MAXFLOAT ) {
      float corr_x, corr_y, rcorr_x, rcorr_y;
      
      corr_x = ((globalSizeX/2.0)+robotPos.x-
		(((extendedSizeX/2)-left)*mapResolution)-ROBOT_INIT_POS_X);
      corr_y = ((globalSizeY/2.0)+robotPos.y-
		(((extendedSizeY/2)-bottom)*mapResolution)-ROBOT_INIT_POS_Y);
      
      rcorr_x = (corr_x+2*ROBOT_INIT_POS_X)/2.0;
      rcorr_y = (corr_y+2*ROBOT_INIT_POS_Y)/2.0;
      
      fprintf( stderr, "*****************************\n" );
      fprintf( stderr, "Map Size: %dx%d\n", map.sizeX, map.sizeY );
      fprintf( stderr, "Extended Map Size: %dx%d\n", extendedSizeX, extendedSizeY );
      fprintf( stderr, "Left: %d\n", left );
      fprintf( stderr, "Right: %d\n", right );
      fprintf( stderr, "Top: %d\n", top );
      fprintf( stderr, "Bottom: %d\n", bottom );
      fprintf( stderr, "Corr - X: %f\n", rcorr_x );
      fprintf( stderr, "Corr - Y: %f\n", rcorr_y );
      fprintf( stderr, "Corr - O: %f\n", 180.0 );
      fprintf( stderr, "*****************************\n" );
      
      fprintf( ofp, "robot_specifications->reposition_robot_initially %d\n", 0);
      fprintf( ofp, "robot_state->correction_type %d\n", 1 );
      fprintf( ofp, "robot_state->correction_parameter_x %f\n", rcorr_x );
      fprintf( ofp, "robot_state->correction_parameter_y %f\n", rcorr_y );
      fprintf( ofp, "robot_state->correction_parameter_angle %f\n", 180.0 );
    }
    
    fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n", globalSizeY);
    fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n", globalSizeX);
    fprintf( ofp, "robot_specifications->resolution %d\n", mapResolution);
    fprintf( ofp, "global_map[0]: %d %d\n", extendedSizeY, extendedSizeX);
    
    for (x = 0; x < left; x++){
      printUnknown(ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
    
    for (x = 0; x < map.sizeX; x++){
      printUnknown( ofp, top);
      for (y = 0; y < map.sizeY; y++)
	if (map.prob[x][y] == -1.0)
	  fprintf(ofp, "-1.0 ");
	else
	  fprintf(ofp, "%.3f ", map.prob[x][y]);
      printUnknown( ofp, bottom);
      fprintf(ofp, "\n");
    }
    
    for (x = 0; x < right; x++){
      printUnknown( ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
    
    fclose(ofp);
  } 
  exit(0);
}


