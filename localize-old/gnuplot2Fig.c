
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/gnuplot2Fig.c,v $
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
 * $Log: gnuplot2Fig.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/10/02 15:16:38  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.1  1998/09/28 21:39:31  fox
 * Converts gnuplot file into xfig.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "graphic.h"
#include "general.h"
#include "probGrid.h"
#include "map.h"
#include "script.h"
#include "proximityTools.h"
#include "allocate.h"
#include "fileNames.h"

/**************************************************************************
**************************************************************************
* Local variables and defines.
 **************************************************************************
 **************************************************************************/

#define MAX_SIZE_X 200
#define MAX_SIZE_Y 100


#define MYMAXCOLORS     155
#define COL_BACK        0                  /* background of window       */
#define COL_UNKNOWN     C_YELLOW           /* color of unknown area      */
#define COL_RED         C_RED
#define COL_BLACK       54
#define COL_WHITE       154

#define COL_ROBODIR    C_BLACK
#define COL_BACKGROUND C_BLUE

#define ESTIMATED_ROB_COLOR C_YELLOW
#define SCRIPT_ROB_COLOR    C_RED

typedef struct{
  int x;
  int y;
} windowPosition;


#define START_GREYSCALE 32
#define NUMBER_OF_GREYSCALES 128
char* greyScales[NUMBER_OF_GREYSCALES];

/* Dynamic window. */
int dynWinMinX=-1, dynWinMinY=-1, dynWinMaxX=-1, dynWinMaxY=-1;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static int
gridWindowScale(int sizeX, int sizeY);

static windowPosition
mapWindowPosition(gridWindow* win, realPosition pos);

static distance
realPositionDistance(realPosition pos1, realPosition pos2)
{
  return (distance) sqrt(fSqr(pos1.x - pos2.x) + fSqr(pos1.y - pos2.y));
}

/*****************************************************************************/
/*   float-Normierung: b = norm (a,...)                                      */
/*****************************************************************************/
static float 
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

static long
round(double x) {
  return (long) floor(x + 0.5);
}



/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

gridWindow *
createWindow(probabilityGrid *m, char* text) {
/* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */

    gridWindow *mapwin;
    
    mapwin = (gridWindow *) malloc(sizeof(gridWindow));
    mapwin->scale = gridWindowScale(m->sizeX, m->sizeY);

    mapwin->gridSizeX = m->sizeX;
    mapwin->gridSizeY = m->sizeY;
    mapwin->gridResolution = m->resolution;
    
    mapwin->sizeX = m->sizeX * mapwin->scale;
    mapwin->sizeY = m->sizeY * mapwin->scale;

    mapwin->firstTimePosition = TRUE;

    EZX_NoMotionEvents();
    mapwin->window = EZX_MakeWindow(text,mapwin->sizeX,mapwin->sizeY,"0,0");
    EZX_SetWindowBackground (mapwin->window,C_RED);
    EZX_Flush();

    mapwin->startX = 0;
    mapwin->startY = 0;
    return(mapwin);
}

void
displayMapWindow( probabilityGrid *m,
		  gridWindow *mapwin)
{
  
/* plots map in window */

  probability maxProb = 0.0;
  probability minProb = 10000.0;
  int x,y;
  
  int winX, winY;
  int endX = mapwin->startX + m->sizeX * mapwin->scale;
  int endY = mapwin->startY + m->sizeY * mapwin->scale;

  for ( x = 0; x < m->sizeX; x++)
    for ( y = 0; y < m->sizeY; y++)
      if ( m->prob[x][y] > maxProb) 
	maxProb = m->prob[x][y];
      else if ( m->prob[x][y] < minProb) 
	minProb = m->prob[x][y];


  y=m->sizeY;
  for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
    y--;
    x=0;
    for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
      if ( m->prob[x][y] <= 0.0 || maxProb == 0.0)
	EZX_SetColor( COL_WHITE);
      else {
	EZX_SetColor( (int) fNorm( m->prob[x][y], minProb, maxProb,
				   COL_WHITE, COL_BLACK));
      }
      EZX_FillRectangle(mapwin->window,winX,winY,
			mapwin->scale,mapwin->scale);
      x++;
    }
  }
  
  EZX_Flush();
  
}



windowPosition
robotWindowPosition(gridWindow* robwin, windowPosition center,
		    realPosition pos) {
    windowPosition winPos;
    winPos.x = 
      robwin->startX
	+ (int) round(((pos.x -
			((int) (center.x - robwin->gridSizeX/2) )
			* robwin->gridResolution)/ robwin->gridResolution + 0.5)
		      * robwin->scale);
    winPos.y =
      robwin->startY
	+ (int) round(((robwin->gridSizeY -
		       (pos.y - ((int) (center.y - robwin->gridSizeY/2) -2 )
			* robwin->gridResolution)/ robwin->gridResolution + 0.5))
		      * robwin->scale);
    return winPos;
}


/********************************************************************
 ********************************************************************
 * Local functions.
 ********************************************************************
 ********************************************************************/


static int
gridWindowScale(int sizeX, int sizeY) {
/*     int scaleX = (int) (1280 / sizeX); */
/*     int scaleY = (int) (960 / sizeY); */
    int scaleX = (int) (800 / sizeX);
    int scaleY = (int) (600 / sizeY);

    if (scaleX < 1)
	scaleX = 1;
    if (scaleY < 1)
        scaleY = 1;

    if (scaleY < scaleX)
	return(scaleY);
    else
	return(scaleX);
}

static windowPosition
mapWindowPosition(gridWindow* win, realPosition pos)
{
  windowPosition winPos;

  winPos.x = win->startX + (int) round((pos.x / win->gridResolution + 0.5) * win->scale);
  winPos.y = win->startY + (int) round((win->gridSizeY - (pos.y / win->gridResolution + 0.5)) * win->scale);
  return winPos;
}

static bool
readProbabilityMap(char *fileName, probabilityGrid *m){
   int x,y;
   float temp;
   char line[MAX_STRING_LENGTH];
   FILE *fp;

   if ((fp = fopen(fileName, "rt")) == NULL) {
     fprintf(stderr,"# Could not open file %s\n", fileName);
     return FALSE;
   }

   fprintf(stderr, "# Reading grid map from %s\n", fileName);
   
   /* seek header for map data in rhino-Type file,
      analyze robot specifications */
   while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL)
	  && (strncmp("global_map[0]", line , 13) != 0)){
     if (strncmp(line,"robot_specifications->resolution",32) == 0) {
	 if (sscanf(&line[32],"%d",&(m->resolution)) != 0 ) {
	 }
     }
     if (strncmp(line,"robot_specifications->autoshifted_x",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetX)) != 0 ) {
	 m->offsetX = m->offsetX;
       }
     }
     if (strncmp(line,"robot_specifications->autoshifted_y",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetY)) != 0 ) {
	 m->offsetY = m->offsetY;
       }
     }
   }
   if (sscanf (line,"global_map[0]: %d %d",&m->sizeY, &m->sizeX)
       == EOF) {
     fprintf(stderr,"ERROR: corrupted file %s\n",fileName);
     fclose(fp);
     return FALSE;
   }
   
   
   m->unknown = -1.0;
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->offsetX = m->offsetY = 0.0;


   
   m->prob = (mapProbability**) malloc(m->sizeX * sizeof(mapProbability*));
   for (x = 0; x < m->sizeX; x++)
     m->prob[x] = (mapProbability*) malloc(m->sizeY * sizeof(mapProbability));

   
   if (m->prob == (mapProbability**) NULL){
      fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n", fileName);
      fclose(fp);
      return FALSE;
   }
   
   for (x=0;x<m->sizeX; x++)
     for (y=0;y<m->sizeY; y++) {
	fscanf(fp,"%e",&temp);
	 if (temp >=  0.0 && temp <= 1.0)
	   m->prob[x][y] = (mapProbability) (temp);
	 else
	   m->prob[x][y] = m->unknown;
      }

   fprintf(stderr, "# done\n");
   return TRUE;
   
}


#define GREYSCALE_STRING "0 %d #%2x%2x%2x\n"
void
dumpGreyScales()
{
  int scale;
  for ( scale = 0; scale < NUMBER_OF_GREYSCALES; scale++) {
    int value = fNorm( scale, 0, NUMBER_OF_GREYSCALES, 255, 0);
    printf( GREYSCALE_STRING, scale + START_GREYSCALE, value, value, value);
  }
}


#define XFIG_SCALE          10
#define XFIG_HEADER         "#FIG 3.1\nPortrait\nCenter\nMetric\n1200 2\n"
#define XFIG_NEW_HEADER         "#FIG 3.2\nPortrait\nCenter\nMetric\nA4\n50.00\nSingle\n-2\n1200 2\n"

#define FRAME_DEPTH 1
#define RECT_DEPTH 2
#define INTENSITY_DEPTH 5

static void
plotXfigHeader( probabilityGrid* map,
		probability* minProb, probability* maxProb)
{
  int x, y;
  int minSet = FALSE;
  int minX=0, maxX=map->sizeX, minY=0, maxY=map->sizeY;
  
  int lastY = XFIG_SCALE * map->sizeY * map->resolution;
  
  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++) 
      if ( map->prob[x][y] > 0) {
	if ( map->prob[x][y] > *maxProb)
	  *maxProb = map->prob[x][y];
	else if ( map->prob[x][y] < *minProb)
	  *minProb = map->prob[x][y];
	if ( ! minSet) {
	  minX = XFIG_SCALE * x * map->resolution;
	  minY = XFIG_SCALE * y * map->resolution;
	  minSet = TRUE;
	}
	maxX = XFIG_SCALE * x * map->resolution;
	maxY = XFIG_SCALE * y * map->resolution;
      }

  printf( XFIG_NEW_HEADER);

  dumpGreyScales();
  
  /* Start of the group. */
  printf( "6 %d %d %d %d\n", minX,
	  lastY - maxY,
	  maxX, lastY - minY);
}

static void
plotXfigCell( int x, int y,
	      probabilityGrid *map,
	      probability min,
	      probability max)
{
#define CELL_FORMAT_STRING  "2 2 0 0 %d %d %d 0 20 0.000 0 0 -1 0 0 5\n       "
#define UNDEFINED_FORMAT_STRING  "2 2 0 1 7 %d 50 0 43 0.000 0 0 -1 0 0 5\n"

  /* Position of lower left corner. */
  int realX = XFIG_SCALE * x * map->resolution;
  int realY = XFIG_SCALE * y * map->resolution;
  int size =  XFIG_SCALE * map->resolution;
  int maxY  = XFIG_SCALE * map->resolution * map->sizeY;

  int intensity = (int) fNorm( map->prob[x][y],
			       min, max,
			       0.0, NUMBER_OF_GREYSCALES-1);
  
  if ( intensity > 0)     
    printf( CELL_FORMAT_STRING, intensity + START_GREYSCALE,
	    intensity + START_GREYSCALE, INTENSITY_DEPTH);
  else {
    if (0)
      printf( UNDEFINED_FORMAT_STRING,
	      NUMBER_OF_GREYSCALES/5 + START_GREYSCALE);
    else
      return;
  }
    
  /* Lower left */
  printf( "%d %d ", realX, maxY - realY);
  
  /* Lower right */
  printf( "%d %d ", realX + size, maxY - realY);
  
  /* Upper right */
  printf( "%d %d ", realX + size, maxY - realY - size);
  
  /* Upper left */
  printf( "%d %d ", realX, maxY - realY - size);
  
  /* Lower left */
  printf( "%d %d\n", realX, maxY - realY);
}


static void
plotXfigRectangle( int minX, int minY,
		   int maxX, int maxY,
		   probabilityGrid *map,
		   int boarderColor,
		   int fillColor)
{
#define RECTANGLE_FORMAT_STRING  "2 2 0 %d %d %d %d 0 %d 0.000 0 0 -1 0 0 5\n       "
#define DYN_WIDTH 3
  
  /* Position of lower left corner. */
  int realMinX = XFIG_SCALE * minX * map->resolution;
  int realMaxX = XFIG_SCALE * maxX * map->resolution;
  int realMinY = XFIG_SCALE * minY * map->resolution;
  int realMaxY = XFIG_SCALE * maxY * map->resolution;
  int size =  XFIG_SCALE * map->resolution;
  int maxYVal= XFIG_SCALE * map->resolution * map->sizeY;

  if ( fillColor > 0)
    printf( RECTANGLE_FORMAT_STRING, DYN_WIDTH, boarderColor, fillColor, RECT_DEPTH, 20);
  else
    printf( RECTANGLE_FORMAT_STRING, DYN_WIDTH, boarderColor, fillColor, RECT_DEPTH, -1);
    
  /* Lower left */
  printf( "%d %d ", realMinX, maxYVal - realMinY);
  
  /* Lower right */
  printf( "%d %d ", realMaxX + size, maxYVal - realMinY);
  
  /* Upper right */
  printf( "%d %d ", realMaxX + size, maxYVal - realMaxY - size);
  
  /* Upper left */
  printf( "%d %d ", realMinX, maxYVal - realMaxY - size);
  
  /* Lower left */
  printf( "%d %d\n", realMinX, maxYVal - realMinY);
}



static void
dumpFrame( probabilityGrid* map)
{
  int ySize = XFIG_SCALE * map->sizeY * map->resolution;
  int xSize = XFIG_SCALE * map->sizeX * map->resolution;
    
  /* y axis */
  printf( "2 1 0 5 -1 -1 %d 0 -1 0.000 0 0 -1 1 0 2\n", FRAME_DEPTH);
  printf( "0 0 4.00 215.00 415.00\n");
  printf( "%d %d %d %d\n",
	  xSize / 2, ySize, xSize / 2, -800);

  /* x axis */
  printf( "2 1 0 5 -1 -1 %d 0 -1 0.000 0 0 -1 1 1 2\n", FRAME_DEPTH);
  printf( "0 0 4.00 215.00 415.00\n");
  printf( "0 0 4.00 215.00 415.00\n");
  printf( "%d %d %d %d\n",
	  -700, ySize, xSize + 700, ySize);

  /* frame */
  printf( "2 2 0 1 0 7 %d 0 -1 0.000 0 0 -1 0 0 5\n", FRAME_DEPTH);
  printf( "%d %d %d %d %d %d %d %d %d %d\n",
	  0,0, xSize,0, xSize,ySize, 0,ySize, 0,0);
  
  /* Names of the axes */
  printf( "4 0 -1 0 0 0 28 0.0000 0 375 1920 %d %d -90 deg/sec\\001\n",
	  - XFIG_SCALE * 20, ySize + 500);
  printf( "4 0 -1 0 0 0 28 0.0000 0 435 2025 %d %d 90 deg/sec\\001\n",
	  xSize - XFIG_SCALE * 150, ySize + 500);
  printf( "4 0 -1 0 0 0 28 0.0000 0 435 2295 %d %d 90 cm/sec\\001\n",
	  xSize / 2 + XFIG_SCALE * 25, -150);

}

static void
dumpXfig( probabilityGrid* map)
{
  int x, y;
  probability max = 0.0;
  probability min = 1000.0;
  probability maxProb = 0.0;

  int maxX=0, maxY=0;
  
  plotXfigHeader( map, &min, &max);

  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++) 
      if ( map->prob[x][y] > map->unknown) {
	plotXfigCell( x, y, map, min, max);

	/* Search for max in dynamic window. */	
	if (  x >= dynWinMinX && x <= dynWinMaxX &&
	      y >= dynWinMinY && y <= dynWinMaxY)
	  if ( map->prob[x][y] > maxProb) {
	    maxX = x;
	    maxY = y;
	    maxProb = map->prob[x][y];
	  }
      }
  
  
  /* Mark the end of the group. */
  printf( "-6\n");
  
  /* Dynamic window. */
  plotXfigRectangle( maxX, maxY,
		     maxX, maxY,
		     map,
		     4, 4);
  plotXfigRectangle( dynWinMinX, dynWinMinY,
		     dynWinMaxX, dynWinMaxY,
		     map,
		     1, -1);

  dumpFrame( map);
  fflush(stdout);
}

void
readValues( char* fileName, probabilityGrid* map)
{
  int x = 0 , y = 0;
  int xCnt = MAX_SIZE_X, yCnt = MAX_SIZE_Y, smooth;
  float dummyX, dummyY, factor;
  FILE* fp;
  char line[BUFFLEN];
  float rVel, tVel, minRVel, maxRVel, minTVel, maxTVel;
  
  if ( (fp = fopen( fileName, "r")) == NULL) {
    fprintf( stderr, "Cannot open %s.\n", fileName);
    exit(-1);
  }

  fprintf( stderr, "Get values from %s ....", fileName);

#define NEW_FILE
#ifdef NEW_FILE
  if ( fscanf( fp, "%d %d %f %d\n", &xCnt, &yCnt, &factor, &smooth) < 4) {
    fprintf( stderr, "Wrong file.\n");
    exit(0);
  }
  else
    fprintf(stderr, "Dimensions: %d %d\n", xCnt, yCnt);

  if ( fscanf( fp, "# %f %f %f %f %f %f\n", &rVel, &tVel,
	       &minRVel, &maxRVel, &minTVel, &maxTVel) < 6) {
    fprintf( stderr, "Wrong file.\n");
    exit(0);
  }
  else {
    fprintf(stderr, "Dynamic window: %f %f [%f : %f] [%f : %f]\n",
	    rVel, tVel, minRVel, maxRVel, minTVel, maxTVel);
  }
#endif	    
  
  /* Set the sizes and allocate memory if not done yet. */
  map->prob = (mapProbability**) allocate2D( xCnt, yCnt, MAP_PROBABILITY);

  
  /* Inner loop. */
  while ( fgets( line, BUFFLEN, fp) != NULL) {
    if ( line[0] == '#')
      fprintf(stderr, "Comment: %s\n", line);
    else if ( sscanf( line, "%f %f %f\n", &dummyX, &dummyY, &(map->prob[x][y])) == 3 ) {
      y++;
      if ( y >= MAX_SIZE_Y) {
	fprintf( stderr, "Too many y values: %d.\n", y);
	exit(0);
      }
      /* Check for the dynamic window. */
      if ( dynWinMinX < 0 && dummyX > minRVel)
	dynWinMinX = x;
      else if ( dynWinMaxX < 0 && dummyX > maxRVel) {
	dynWinMaxX = x;
	fprintf(stderr, "rvel %f %d %d\n", dummyX, dynWinMinX, dynWinMaxX);
      }
	
      if ( dynWinMinY < 0 && dummyY > minTVel)
	dynWinMinY = y;
      else if ( dynWinMaxY < 0 && dummyY > maxTVel) {
	dynWinMaxY = y;
	fprintf(stderr, "tvel %f %d %d\n", dummyY, dynWinMinY, dynWinMaxY);
      }
    }
    else {
      if ( yCnt != MAX_SIZE_Y && yCnt != y)
	fprintf(stderr, "Oops. Number of y values has changed %d -> %d.\n",
		yCnt, y);
      else
	yCnt = y;
      y = 0;
      x++;
      
      if ( x >= MAX_SIZE_X) {
	fprintf( stderr, "Too many x values: %d.\n", x);
	exit(0);
      }
    }
  }
  xCnt = x;
  
  fprintf(stderr, "done (%d %d values).\n",
	  xCnt, yCnt);

  map->sizeX = xCnt;
  map->sizeY = yCnt;
  map->resolution = 10.0;
}


int
main( int argc, char** argv)

{
  int i;
  int useGraphics = FALSE;
  int readMap = FALSE;
  probabilityGrid map;
  

  /* Read the values into the map structure. */
  if ((argc < 2) || (argc > 3))
    {
      fprintf(stderr, "usage: %s gnuplotfile [-map] [-graph]\n", argv[0]);
      exit(0);
    }      

  for (i=2; i<argc; i++) {
    if ((strcmp(argv[i],"-graph")==0)) {
      useGraphics = TRUE;
    }
    else if ((strcmp(argv[i],"-map")==0)) {
      readMap = TRUE;
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[i]);
      getchar();
    }
  }
    
  /* Try to get the values. */
  if ( ! readMap)
    readValues( argv[1], &map);
  else
    readProbabilityMap( argv[1], &map);
    

  /* Display the probabilities. */
  if (useGraphics) {
    gridWindow* win = NULL;

    /* Create the window. */
    if ( win == NULL)
      win = createWindow( &map, "plots");
    displayMapWindow( &map, win);
    getchar();
  }
  else
    dumpXfig( &map);
  
  exit(0);
}

