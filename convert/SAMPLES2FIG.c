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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/SAMPLES2FIG.c,v $
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
 * $Log: SAMPLES2FIG.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1999/09/29 16:44:19  fox
 * Newest version.
 *
 * Revision 1.2  1999/09/17 21:49:44  fox
 * KI-99 version.
 *
 * Revision 1.1  1999/09/09 08:06:17  fox
 * Generates animations.
 *
 * Revision 1.14  1999/09/03 23:04:52  fox
 * Updated sample set format.
 *
 * Revision 1.13  1999/06/04 05:45:13  wolfram
 * Fixed a bug with round-off errors
 *
 * Revision 1.12  1999/06/03 15:07:42  fox
 * Merged tree version with run-length encoding.
 *
 * Revision 1.11  1999/06/03 12:23:23  wolfram
 * Added run-lenght compression for clipped maps and fixed bugs.
 *
 * Revision 1.10  1999/05/03 18:55:35  fox
 * Several changes for KI graphics.
 *
 * Revision 1.9  1999/04/27 14:52:09  fox
 * Better support for trees and samples (map not mandatory any more).
 *
 * Revision 1.8  1999/04/19 16:31:02  fox
 * Added tree support.
 *
 * Revision 1.7  1999/03/20 20:34:58  wolfram
 * Fixed a bug
 *
 * Revision 1.6  1999/03/18 18:12:43  fox
 * Added feature to zoom into the map, read samples and set max intensity of
 * the map.
 *
 * Revision 1.5  1999/01/22 00:32:49  wolfram
 * MAP2FIG now reads positionlogs without time stamp
 *
 * Revision 1.4  1998/11/09 21:34:16  wolfram
 * Added sensings to trajectory including a -nomap option
 *
 * Revision 1.3  1998/10/26 22:17:32  wolfram
 * Added clipped occupancy grid maps (in blue)
 *
 * Revision 1.2  1998/10/26 08:53:03  wolfram
 * Fixed bugs, added things
 *
 * Revision 1.1  1998/10/20 17:30:50  fox
 * Converts a map file into fig format. Useless for large maps.
 *
 * Revision 1.11  1998/02/16 15:04:56  wolfram
 * The map is now written as a grouped object
 *
 * Revision 1.10  1998/01/12 23:12:01  wolfram
 * Added reference position support
 *
 * Revision 1.9  1997/12/15 08:58:02  fox
 * Fixed a bug.
 *
 * Revision 1.8  1997/12/12 16:23:40  fox
 * Minor changes.
 *
 * Revision 1.7  1997/12/05 11:53:51  fox
 * Changed intensity handling.
 *
 * Revision 1.6  1997/12/02 10:43:17  fox
 * Added some features.
 *
 * Revision 1.5  1997/11/28 13:08:46  fox
 * Added specification of height.
 *
 * Revision 1.4  1997/06/03 11:49:12  fox
 * Museum version.
 *
 * Revision 1.3  1997/04/27 16:29:54  wolfram
 * Updates to new version of SIMULATOR_II
 *
 * Revision 1.2  1997/01/18 11:29:38  wolfram
 * *** empty log message ***
 *
 * Revision 1.1  1997/01/18 11:28:16  wolfram
 * SIM2FIG converts simulator maps to XFIG files
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define MAX_STRING_LENGTH 255

#define XFIG_BLUE 1
#define XFIG_GREEN 2
#define XFIG_CYAN 3
#define XFIG_RED 4
#define XFIG_MAGENTA 5
#define XFIG_YELLOW 6
#define XFIG_BROWN 24
#define XFIG_PINK 28
#define XFIG_BLACK 0
#define XFIG_WHITE 7

#define MAX_NUMBER_OF_OBJECTS 10000
#define MAP_MARK "MAP"
#define RECTANGLE_MARK "RECTANGLE"
#define DOOR_MARK "DOOR"
#define ROBOT_MARK "ROBOT"
#define CUBE_MARK "CUBE"
#define CYLINDER_MARK "CYLINDER"
#define SKIP 3

#define BUFFLEN 1024

#define TRUE 1
#define FALSE 0
#define CUBE 0
#define CYLINDER 1

#define MAX_GREY_SCALE 255
int drawFrame = 0;
int blackAndWhite = 0;
#define START_GREYSCALE 32
#define NUMBER_OF_GREYSCALES 200
char* greyScales[NUMBER_OF_GREYSCALES];
float densityDarkness = NUMBER_OF_GREYSCALES;
float mapDarkness = 0;
float globalMinDarkness = 0;

#define GREY 0
#define BLUE 1


typedef struct{
  float x;
  float y;
} point;

typedef struct{
  float centerX;
  float centerY;
  float centerZ;
  float width;
  float depth;
  float height;
  float rot;
  int type;
} rectangle;

/* This struct contains the given position / occupancy probabilities. */
typedef struct {
  float unknown;
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
  float** prob;
} probabilityGrid;

static int headerPlotted = FALSE;

/* Sample set related stuff. */
#define NUMBER_KEY_WORD "#Number "
#define DIMENSION_KEY_WORD "#Dimension "
#define MIN_KEY_WORD "#Min "
#define MAX_KEY_WORD "#Max "
#define DENSITY_KEY_WORD "#MaxDensity "
#define REF_POS_KEY_WORD "#RefPosition "
#define TIME_STAMP_KEY_WORD "#Time "

#define MAX_NUMBER_OF_ROBOTS 10
#define MAX_NUMBER_OF_SETS_PER_ROBOT 200
#define MAX_NUMBER_OF_SAMPLES 2000

#define TIMESKIP 1.0
static FILE* whirlFile = NULL;

point* samplePoints[MAX_NUMBER_OF_ROBOTS][MAX_NUMBER_OF_SETS_PER_ROBOT];
struct timeval sampleTimeStamps[MAX_NUMBER_OF_ROBOTS][MAX_NUMBER_OF_SETS_PER_ROBOT];
int    numberOfSamplePoints[MAX_NUMBER_OF_ROBOTS][MAX_NUMBER_OF_SETS_PER_ROBOT];
point  sampleRobotPositions[MAX_NUMBER_OF_ROBOTS][MAX_NUMBER_OF_SETS_PER_ROBOT];
float  sampleRobotOrientations[MAX_NUMBER_OF_ROBOTS][MAX_NUMBER_OF_SETS_PER_ROBOT];
char*  robotName[MAX_NUMBER_OF_ROBOTS];
int    hasSeen[MAX_NUMBER_OF_ROBOTS][MAX_NUMBER_OF_SETS_PER_ROBOT];

int depth = 0;

/* The borders of the area to be displayed. */
float minXToDump, minYToDump, maxXToDump, maxYToDump;
int minBorderSet = 0, maxBorderSet = 0;

/* The borders of the defined area. */
float yZeroLine;

static void
dumpFrame(float scaleFactor, FILE* fp);

#define MIN(x,y)  ((x) > (y) ? (y) : (x))
#define MAX(x,y)  ((x) < (y) ? (y) : (x))

/* Checks whether a coordinate is within the borders set with command
 * line arguments min and max. */
static int
inBorders( float x, float y)
{
  return ( ( ! minBorderSet ||
	     ((x >= minXToDump) &&
	      (y >= minYToDump)))
	   &&
	   ( ! maxBorderSet ||
	     ((x <= maxXToDump) &&
	      (y <= maxYToDump))));
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

static int
xfigGrey( float value, float min, float max, float minDarkness)
{
  int greyScaleOffset;
  
  /* Is this a color to be scaled into the whole grey colors? */
  if ( minDarkness == globalMinDarkness) 
    greyScaleOffset = (int) fNorm( value, min, max, 0.0, NUMBER_OF_GREYSCALES - 1);
  else {
    int ng = MAX_GREY_SCALE;
    int numberOfGreyScales =
      ng * ( (float) (ng - minDarkness) / (float) (ng - globalMinDarkness));
    greyScaleOffset = (int) fNorm( value, min, max, 0.0, numberOfGreyScales);
  }

  if ( greyScaleOffset < 0 || greyScaleOffset >= NUMBER_OF_GREYSCALES) {
    fprintf( stderr, "ERROR: %f [%f : %f] [%f %f] --> %d\n",
	     value, min, max, minDarkness, globalMinDarkness, greyScaleOffset);
  }

  return START_GREYSCALE + greyScaleOffset;
}


static int
shrinkMap(probabilityGrid *m)
{
  register int x,y;
  register int empty = TRUE;

  int min_x,max_x,min_y,max_y;

  int oldsizeX;

  for (min_x = 0; min_x < m->sizeX && empty; min_x++)
    for (y=0; y < m->sizeY; y++)
      empty = empty && (m->prob[min_x][y] == m->unknown);
  min_x--;

  empty = TRUE;
  for (max_x = m->sizeX-1; max_x >=0 && empty; max_x--)
    for (y=0; y < m->sizeY; y++)
      empty = empty && (m->prob[max_x][y] == m->unknown);
  max_x++;
  
  empty = TRUE;
  for (min_y = 0; min_y < m->sizeY && empty; min_y++)
    for (x=0; x < m->sizeX; x++)
      empty = empty && (m->prob[x][min_y] == m->unknown);
  min_y--;
    
  empty = TRUE;
  for (max_y = m->sizeY-1; max_y >=0 && empty; max_y--)
    for (x=0; x < m->sizeX; x++)
      empty = empty && (m->prob[x][max_y] == m->unknown);
  max_y++;
  
  oldsizeX = m->sizeX;
  
  m->sizeX = max_x - min_x + 1;
  m->sizeY = max_y - min_y + 1;

  m->shiftedX = min_x;
  m->shiftedY = min_y;
  
  fprintf(stderr,
	  "# Map shrinked: origin %d %d, size %d %d, resolution %d\n",
	  min_x, min_y,
	  m->sizeX, m->sizeY,
	  m->resolution);

  for(x=0; x<m->sizeX; x++)
    for(y=0; y<m->sizeY; y++)
      m->prob[x][y] = m->prob[x+min_x][y+min_y];

  
  /*  m->offsetX += m->resolution * min_x;
  m->offsetY += m->resolution * min_y; */
  m->offsetX = m->offsetY = 0;
  m->maxRealX = m->offsetX + m->sizeX * m->resolution;
  m->maxRealY = m->offsetY + m->sizeY * m->resolution;
  
  


  return(1);
  
}



static int
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
   
   
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->offsetX = m->offsetY = 0.0;
   m->unknown = -1.0;

   
   m->prob = (float**) malloc(m->sizeX * sizeof(float*));
   for (x = 0; x < m->sizeX; x++)
     m->prob[x] = (float*) malloc(m->sizeY * sizeof(float));

   
   if (m->prob == (float**) NULL){
      fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n", fileName);
      fclose(fp);
      return FALSE;
   }
   
   for (x=0;x<m->sizeX; x++)
     for (y=0;y<m->sizeY; y++) {
	fscanf(fp,"%e",&temp);
	 if (temp >=  0.0 && temp <= 1.0)
	   m->prob[x][y] = (float) (temp);
	 else
	   m->prob[x][y] = m->unknown;
      }

   shrinkMap(m);
   fprintf(stderr, "# done\n");

   return TRUE;
}

/* Adjust the borders of the display area. */
static void
adjustDumpArea( probabilityGrid *map)
{
  if ( ! minBorderSet) {
    minXToDump = map->offsetX;
    minYToDump = map->offsetY;
    minBorderSet = TRUE;
  }
  if ( ! maxBorderSet) {
    maxXToDump = map->offsetX + (map->sizeX + 1) * map->resolution;
    maxYToDump = map->offsetY + (map->sizeY + 1) * map->resolution;
    maxBorderSet = TRUE;
  }
  fprintf( stderr, "# Borders: %f - %f   %f - %f\n",
	   minXToDump, maxXToDump,
	   minYToDump, maxYToDump);
}


int
round(float x)
{
  return (int) (x + 0.5);
}



#define GREYSCALE_STRING "0 %d #%2x%2x%2x\n"
void
dumpGreyScales( FILE* fp)
{
  int scale;
  for ( scale = 0; scale < NUMBER_OF_GREYSCALES; scale++) {
    int value = (int) fNorm( scale, 0, NUMBER_OF_GREYSCALES, MAX_GREY_SCALE, globalMinDarkness);
    fprintf( fp, GREYSCALE_STRING, scale + START_GREYSCALE, value, value, value);
  }
}


#define XFIG_NEW_HEADER     "#FIG 3.2\nPortrait\nCenter\nMetric\nA4\n50.00\nSingle\n-2\n1200 2\n"

#define FRAME_DEPTH 1
#define RECT_DEPTH 4
#define TREE_DEPTH 3
#define DENSITY_DEPTH 6
#define INTENSITY_DEPTH 5
#define SAMPLE_DEPTH 9

static void
dumpCompoundBorders( float scaleFactor, FILE* fp)
{
  fprintf( fp, "6 %d %d %d %d\n", 0, 0,
	   round( scaleFactor * (maxXToDump - minXToDump)),
	   round( scaleFactor * (maxYToDump - minYToDump)));
}

static void
plotXfigHeader( float scaleFactor, FILE* fp)
{
  if ( headerPlotted)
    return;
  
  fprintf( fp, XFIG_NEW_HEADER);
  
  dumpGreyScales( fp);

  if (drawFrame) dumpFrame( scaleFactor, fp);

  headerPlotted = TRUE;
}

static void
plotXfigMapCell( float prob, float size,
		 float minX, float minY,
		 float minVal, float maxVal,
		 int numberOfXCells,
		 float scaleFactor, int type, FILE* fp,
		 int color)
{
#define CELL_FORMAT_STRING  "2 2 0 0 %d %d %d 0 20 0.000 0 0 -1 0 0 5\n       "
#define UNDEFINED_FORMAT_STRING  "2 2 0 1 7 %d 50 0 43 0.000 0 0 -1 0 0 5\n"
  
  /* Position of lower left corner. */
  float realX = scaleFactor * ( minX - minXToDump);
  float realY = scaleFactor * (maxYToDump - minY);
  float SizeX, SizeY;

  size *= scaleFactor;
  SizeX = size * numberOfXCells;
  SizeY = size;
  
  if (type == GREY){

    int intensity = xfigGrey( 1.0 - prob, minVal, maxVal, mapDarkness);  

    if ( intensity > START_GREYSCALE)     
      fprintf( fp, CELL_FORMAT_STRING, intensity, intensity, INTENSITY_DEPTH);
    else {
      return;
    }
  }
  else if (type == BLUE)
    if ( color >= 0)
      fprintf( fp, CELL_FORMAT_STRING, color, color, INTENSITY_DEPTH);
    else if ( blackAndWhite) 
      fprintf( fp, CELL_FORMAT_STRING, XFIG_BLACK, XFIG_BLACK, INTENSITY_DEPTH);
    else {
      int intensity = xfigGrey( 0.3, minVal, maxVal, mapDarkness);  
      fprintf( fp, CELL_FORMAT_STRING, intensity, intensity, INTENSITY_DEPTH);
      /*        fprintf( fp, CELL_FORMAT_STRING, XFIG_BLUE, XFIG_BLUE, INTENSITY_DEPTH); */
    }
  else {
    fprintf( stderr, "ERROR: only GREY and BLUE type supported.\n");
    exit(0);
  }
  
  /* Lower left */
  fprintf( fp, "%d %d ", round(realX), round(realY));
  
  /* Lower right */
  fprintf( fp, "%d %d ", round(realX + SizeX), round(realY));
  
  /* Upper right */
  fprintf( fp, "%d %d ", round(realX + SizeX), round(realY - SizeY));
  
  /* Upper left */
  fprintf( fp, "%d %d ", round(realX), round(realY - SizeY));
  
  /* Lower left */
  fprintf( fp, "%d %d\n", round(realX), round(realY));
}


static void
plotXfigRectangle( float minX, float minY,
		   float maxX, float maxY,
		   int borderColor,
		   int fillColor, float scaleFactor,
		   int depth, FILE* fp)
{
#define RECTANGLE_FORMAT_STRING  "2 2 0 %d %d %d %d 0 %d 0.000 0 0 -1 0 0 5\n       "
#define DYN_WIDTH 1
  
  /* Position of lower left corner. */
  int realMinX = round(scaleFactor * (minX - minXToDump));
  int realMinY = round(scaleFactor * (maxYToDump - minY));
  int realMaxX = round(scaleFactor * (maxX - minXToDump));
  int realMaxY = round(scaleFactor * (maxYToDump - maxY));

  if ( fillColor > 0)
    fprintf( fp, RECTANGLE_FORMAT_STRING, 0, borderColor, fillColor, depth, 20);
  else
    fprintf( fp, RECTANGLE_FORMAT_STRING, DYN_WIDTH, borderColor, fillColor, depth, -1);
    
  /* Lower left */
  fprintf( fp, "%d %d ", realMinX, realMinY);
  
  /* Lower right */
  fprintf( fp, "%d %d ", realMaxX, realMinY);
  
  /* Upper right */
  fprintf( fp, "%d %d ", realMaxX, realMaxY);
  
  /* Upper left */
  fprintf( fp, "%d %d ", realMinX, realMaxY);
  
  /* Lower left */
  fprintf( fp, "%d %d\n", realMinX, realMinY);
}



static void
dumpFrame(float scaleFactor, FILE* fp)
{
  int xSize = round( (maxXToDump-minXToDump) * scaleFactor);
  int ySize = round( (maxYToDump-minYToDump) * scaleFactor);

  /* frame */
  fprintf( fp, "2 2 0 1 0 7 %d 0 -1 0.000 0 0 -1 0 0 5\n", FRAME_DEPTH);
  fprintf( fp, "%d %d %d %d %d %d %d %d %d %d\n",
	  0, 0, xSize, 0,
	  xSize, ySize, 0, ySize, 0, 0);
}

static void
dumpMap( probabilityGrid* map, float scaleFactor, int clip,
	 float thresh, int merge, FILE* fp, int detection)
{
  int x, y;
  float maxProb = 0.0;
  float minProb = 1000.0;
  int color = -1;
  
  float sizeOfCell =  merge * map->resolution;

  plotXfigHeader( scaleFactor, fp);

  dumpCompoundBorders( scaleFactor, fp);

  if ( ! clip) {
    
    /* Determine min and max probability. */
    for ( x = 0; x <= map->sizeX - merge; x+=merge)
      for ( y = 0; y <= map->sizeY - merge; y+=merge)
	if ( inBorders( x * map->resolution, y * map->resolution) &&
	     inBorders( (x+merge) * map->resolution, (y+merge) * map->resolution)) {
	  
	  /* Merge over several cells. */
	  int numberOfMergedCells = 0;
	  float avgProb = 0.0;
	  int mergeX, mergeY;
	  
	  for ( mergeX = 0; mergeX < merge; mergeX++) {
	    
	    int tmpX = x + mergeX;
	    
	    for ( mergeY = 0; mergeY < merge; mergeY++) {
	      
	      int tmpY = y + mergeY;
	      
	      if ( map->prob[tmpX][tmpY] > 0) {
		avgProb += map->prob[tmpX][tmpY];
		numberOfMergedCells++;
	      }
	    }
	  }
	  if ( numberOfMergedCells > 0) {
	    avgProb /= numberOfMergedCells;
	    
	    if ( avgProb > 0) {
	      if ( avgProb > maxProb)
		maxProb = avgProb;
	      else if ( avgProb < minProb)
		minProb = avgProb;
	    }
	  }
	}
    
    /* Now dump each grid cell. */
    /* Determine min and max probability. */
    for ( x = 0; x <= map->sizeX - merge; x+=merge) {
      for ( y = 0; y <= map->sizeY - merge; y+=merge) {
	if ( inBorders( x * map->resolution, y * map->resolution) &&
	     inBorders( (x+merge) * map->resolution, (y+merge) * map->resolution)) {
	  
	  /* Merge over several cells. */
	  int numberOfMergedCells = 0;
	  float avgProb = 0.0;
	  int mergeX, mergeY;
	  
	  for ( mergeX = 0; mergeX < merge; mergeX++) {
	    
	    int tmpX = x + mergeX;
	    
	    for ( mergeY = 0; mergeY < merge; mergeY++) {

	      int tmpY = y + mergeY;
	      
	      if ( map->prob[tmpX][tmpY] >= 0) {
		avgProb += map->prob[tmpX][tmpY];
		numberOfMergedCells++;
	      }
	    }
	  }
	  if ( numberOfMergedCells > 0) {
	    
	    avgProb /= numberOfMergedCells;
	    
	    if ( avgProb > map->unknown) {
	      plotXfigMapCell( avgProb, sizeOfCell,
			       x * map->resolution, y * map->resolution,
			       minProb, maxProb,
			       1, scaleFactor, GREY, fp, color);
	    }
	  }
	}
      }
    }
  }
  /* Binary map: Perform run-length encoding */
  else {
    for ( y = 0; y < map->sizeY;y++) {
      for ( x = 0; x < map->sizeX;){
	if ( inBorders( x * map->resolution, y * map->resolution)){
	  if (map->prob[x][y] < thresh || map->prob[x][y] == map->unknown){
	    int lengthX = 1;
	    while ((x + lengthX) < map->sizeX &&
		   inBorders( (x + lengthX)* map->resolution,
			      y * map->resolution) &&
		   (map->prob[x+lengthX][y] < thresh
		    || map->prob[x+lengthX][y] == map->unknown)){
	      lengthX++;
	    }
	    if ( 0 && detection) {
	      color = xfigGrey( 0.7, 0, 1, mapDarkness);  
/*  	      color = XFIG_BLUE; */
	    }
	    else {
	      color = xfigGrey( 0.3, 0, 1, mapDarkness);  
	    }

	    /* Most of the parameters are not needed in this case. */
	    plotXfigMapCell( 0.0, sizeOfCell,
			     x * map->resolution, y * map->resolution,
			     0.0, 1.0, lengthX, scaleFactor, BLUE, fp,
			     color);
	    if (y == map->sizeY - 1) fprintf(stderr, "%d %d %d\n", x, y, lengthX);
	    x = x + lengthX;
	  }
	  else x++;
	}
	else x++;
      }
    }  
  }
  
  /* Mark the end of the group. */
  fprintf( fp, "-6\n");
  
  fflush(stdout);
}

static void
printTime( char* infoString, struct timeval t)
{
  if( infoString)
    fprintf(stderr, "%s %ld %ld\n", infoString, t.tv_sec, t.tv_usec);
  else
    fprintf(stderr, "%ld %ld\n", t.tv_sec, t.tv_usec);
}


static int
before( struct timeval t1, struct timeval t2)
{
  float diff;
  
  diff =  (float) (t1.tv_usec - t2.tv_usec) / 1000000.0;
  diff += (float) (t1.tv_sec - t2.tv_sec);
  
  return diff <= 0.0;
}

static void
unzip( char* fileName)
{
  char command[255];
  sprintf( command, "gunzip -f %s.gz", fileName);
  system( command);
}

static void
zip( char* fileName)
{
  char command[255];
  sprintf( command, "gzip -f %s", fileName);
  system( command);
}

static void
dumpSampleFrame( int* currentSampleSet, int numberOfRobots, float scaleFactor,
		 probabilityGrid* map,int clip,
		 float thresh, int merge)
{
  static int frameNo = 0;
    
  int intPosX, intPosY;
  int sampleRadius = scaleFactor;
  int robot;
  int color, detection = FALSE;
  int sample;
  int hasBeenSeen[MAX_NUMBER_OF_ROBOTS];

  FILE* fp;
  char fileName[255];

  /* Check which of the robots has been detected. */
  for ( robot = 0; robot < numberOfRobots; robot++)
    hasBeenSeen[robot] = 0;
  for ( robot = 0; robot < numberOfRobots; robot++) {
    int currSet = currentSampleSet[robot];
    if ( hasSeen[robot][currSet] >= 0) {
      detection = TRUE;
      hasBeenSeen[hasSeen[robot][currSet]] = 1;
      fprintf(stderr, "%d saw %d\n", robot, hasSeen[robot][currSet]);
    }
  }

/*    if ( ! detection) { */
/*      frameNo++; */
/*      return; */
/*    } */
    
  /* File for whirlgif. */
  if ( whirlFile == NULL) {
    whirlFile = fopen("animate", "w");
    if ( whirlFile == NULL) {
      fprintf( stderr, "ERROR: cannot open whirl file %s.\n", "animate");
      exit(0);
    }
    fprintf( whirlFile, "~/tools/animateGifs/whirlgif -g -o animation.gif ");
  }
  
  
  /* If there is a detection event then we want to stop the animation. */
  if ( detection)
    fprintf( whirlFile, " -time %d", (int) (TIMESKIP * 200));
  else
    fprintf( whirlFile, " -time %d", (int) (TIMESKIP * 50));

  if ( frameNo < 10) {
    sprintf(fileName, "frame.00%d.fig", frameNo);
    fprintf(whirlFile, " frame.00%d.gif", frameNo);
  }
  else if ( frameNo < 100) {
    sprintf(fileName, "frame.0%d.fig", frameNo);
    fprintf(whirlFile, " frame.0%d.gif", frameNo);
  }
  else {
    sprintf(fileName, "frame.%d.fig", frameNo);
    fprintf(whirlFile, " frame.%d.gif", frameNo);
  }

  if ((fp = fopen(fileName,"wt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open gif file '%s'!\n",fileName);
    exit(0);
  }

  fprintf(stderr, "Dump frame %d.\n", frameNo);

  plotXfigHeader( scaleFactor, fp);
  
  for ( robot = 0; robot < numberOfRobots; robot++) {

    int currSet = currentSampleSet[robot];
    int robRadius = 50 * scaleFactor;
    
    if ( blackAndWhite && robot == 0)
      color = XFIG_BLACK;
    else if ( blackAndWhite && robot == 1)
      color = XFIG_GREEN;
    else {
      switch (robot) {
      case 0: 
	color = XFIG_RED; break;
      case 1: 
	color = XFIG_BLUE; break;
      case 2: 
	color = XFIG_YELLOW; break;
      case 3: 
	color = XFIG_GREEN; break;
      case 4: 
	color = XFIG_CYAN; break;
      case 5: 
	color = XFIG_MAGENTA; break;
      case 6: 
	color = XFIG_PINK; break;
      case 7: 
	color = XFIG_BROWN; break;
      default:
	color = XFIG_BLACK; break;
      }
    }
    
    fprintf( stderr, "Start dumping robot %d ... ", robot);
    
    /* Header to dump the samples as compund object. */
    dumpCompoundBorders( scaleFactor,fp);
    
    for ( sample = 0; sample < numberOfSamplePoints[robot][currSet];
	  sample++) {
      
      intPosX = round( (samplePoints[robot][currSet][sample].x - minXToDump) * scaleFactor);
      intPosY = round((maxYToDump - samplePoints[robot][currSet][sample].y) * scaleFactor);
      
      /* The number of the sample set determines the depth. */
      fprintf( fp, "1 3 0 1 %d 0 %d 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
	       color, SAMPLE_DEPTH,
	       intPosX, intPosY, sampleRadius, sampleRadius, intPosX, intPosY,
	       intPosX + sampleRadius, intPosY + sampleRadius);
    }
    
    /* End of compound object. */
    fprintf( fp, "-6\n");
    
    /* Dump the reference position. */
    intPosX = round((sampleRobotPositions[robot][currSet].x - minXToDump) * scaleFactor);
    intPosY = round((maxYToDump - sampleRobotPositions[robot][currSet].y) * scaleFactor);

    if ( hasSeen[robot][currSet] >= 0) {

      robRadius *= 3;

      fprintf(stderr, "Robot %d has detected %d in set %d.\n", robot,
	      hasSeen[robot][currSet], currSet);
      fprintf( fp, "1 3 0 3 %d %d 1 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
	       XFIG_BLACK, color,
	       intPosX, intPosY, robRadius, robRadius, intPosX, intPosY,
	       intPosX + robRadius, intPosY + robRadius);

      /* Each detection should only be shown once. */
      hasSeen[robot][currSet] = -1;      
    }
    else if ( hasBeenSeen[robot]) {
      robRadius *= 2;
      
      fprintf(stderr, "Robot %d has BEEN detected in set %d.\n", robot, currSet);
      fprintf( fp, "1 3 0 3 %d %d 1 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
	       XFIG_BLACK, color,
	       intPosX, intPosY, robRadius, robRadius, intPosX, intPosY,
	       intPosX + robRadius, intPosY + robRadius);
    }
    else if ( 1)
      fprintf( fp, "1 3 0 2 %d %d 1 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
	       XFIG_BLACK, color,
	       intPosX, intPosY, robRadius, robRadius, intPosX, intPosY,
	       intPosX + robRadius, intPosY + robRadius);
    
    else
      fprintf( fp, "1 3 0 3 %d %d 1 0 -1 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
	       color, color,
	       intPosX, intPosY, robRadius, robRadius, intPosX, intPosY,
	       intPosX + robRadius, intPosY + robRadius);
    
    fprintf( fp, "2 1 0 2 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n  %d %d %d %d\n",
	     intPosX, intPosY,
	     intPosX + (int) (robRadius * (float) cos(sampleRobotOrientations[robot][currSet])),
	     intPosY - (int) (robRadius * (float) sin(sampleRobotOrientations[robot][currSet])));
    
    fprintf( stderr, "done.\n");
  }

  if ( map) 
    dumpMap( map, scaleFactor, clip, thresh, merge, fp, detection);

  /* Reset the detection. */
  for ( robot = 0; robot < numberOfRobots; robot++) 
    hasSeen[robot][currentSampleSet[robot]] = -1;
  
  headerPlotted = FALSE;
  fclose(fp);
  fflush(whirlFile);
  zip( fileName);
  frameNo++;
}


static int
numberOfRobot( char* name, int numberOfRobots)
{
  int robot;
  
  for ( robot = 0; robot < numberOfRobots; robot++) {
    if ( strcmp( name, robotName[robot]) == 0) {
      return robot;
    }
  }
  return -1;
}

/* Dumps the sample sets synchronized by time. */
static void
dumpSamples( int numberOfRobots, float scaleFactor,
	     probabilityGrid* map,int clip,
	     float thresh, int merge)
{
  int robot;
  int currentSampleSet[MAX_NUMBER_OF_ROBOTS];
  struct timeval synchroTime;
  int timeInitialized = FALSE;
  int notYetFinished = TRUE;
  
  /* Get the latest start time. */
  for ( robot = 0; robot < numberOfRobots; robot++) {

    currentSampleSet[robot] = 0;
    
    if ( samplePoints[robot][0]) {
      if ( ! timeInitialized) {
	synchroTime = sampleTimeStamps[robot][0];
	timeInitialized = TRUE;
      }
      else if ( before( synchroTime, sampleTimeStamps[robot][0])) {
	synchroTime = sampleTimeStamps[robot][0];
      }
    }
  }

  printTime( "Start time ", synchroTime);
  
  while( notYetFinished) {

    notYetFinished = FALSE;

    printTime( "Look for new sets ", synchroTime);
    
    /* Increase sample set according to synchro time. */
    for ( robot = 0; robot < numberOfRobots; robot++) {
      
      int newerSetFound = FALSE;
      int sampleSet = currentSampleSet[robot];
      int nextSampleSet = currentSampleSet[robot];
      //      int oldestSampleSetBeforeSynchro = currentSampleSet[robot];

      fprintf(stderr, "ROBOT %d\n", robot);
      
      /* Look for the sample set immediately before the synchro time. */
      while ( sampleSet < MAX_NUMBER_OF_SETS_PER_ROBOT && ! newerSetFound) {
	if ( samplePoints[robot][sampleSet]) {
	  if ( hasSeen[robot][sampleSet] >= 0) {
	    fprintf(stderr, "DETECTION ");
	    printTime(NULL, sampleTimeStamps[robot][nextSampleSet]);
	    synchroTime = sampleTimeStamps[robot][nextSampleSet];
	    nextSampleSet = sampleSet;
	    newerSetFound = TRUE;
	  }
	  else if ( before( sampleTimeStamps[robot][nextSampleSet], synchroTime)) {
	    fprintf(stderr, "BEFORE %d %d ", sampleSet, nextSampleSet);
	    printTime(NULL, sampleTimeStamps[robot][nextSampleSet]);
	    nextSampleSet = sampleSet;
	  }
	  else {
	    fprintf(stderr, "AFTER %d %d ", sampleSet, nextSampleSet);
	    printTime(NULL, sampleTimeStamps[robot][nextSampleSet]);
	    newerSetFound = TRUE;
	  }
	}
	else
	  fprintf( stderr, "Sample set %d not defined.\n", sampleSet);
	sampleSet++;
      }
      if ( sampleSet < MAX_NUMBER_OF_SETS_PER_ROBOT) {
	if ( currentSampleSet[robot] != nextSampleSet) {
	  fprintf(stderr, "FOUND NEw %d\n", nextSampleSet);
	  currentSampleSet[robot] = nextSampleSet;
	}
      }
      if ( newerSetFound) 
	notYetFinished = TRUE;
    }

    if ( notYetFinished) {
      dumpSampleFrame( currentSampleSet, numberOfRobots, scaleFactor,
		       map, clip, thresh, merge);
    }
    
    synchroTime.tv_sec += TIMESKIP;
  }
}

/* Reads in all sample sets and stores the points in the global structures. */
static void
readDetections( char* detectionFile, int numberOfRobots)
{
  FILE *fp;
  char line[BUFFLEN];
  char observer[BUFFLEN];
  char detected[BUFFLEN];
  int observerSet, detectedSet;
  int robot, set;
  
  /* Initialize the detections. */
  for ( robot = 0; robot < MAX_NUMBER_OF_ROBOTS; robot++)
    for (set = 0; set < MAX_NUMBER_OF_SETS_PER_ROBOT; set++) {
      /*        if ( set == 2) */
      /*  	if ( robot != 1) */
      /*  	  hasSeen[robot][set] = 1; */
      /*  	else */
      hasSeen[robot][set] = -1;
    }
  
  if ( detectionFile == NULL || (fp = fopen( detectionFile, "r")) == NULL) {
    fprintf(stderr,"ERROR: Could not open detection file '%s'!\n",
	    detectionFile);
    return;
  }

  while (!feof(fp)) {
		     
    fgets(line, BUFFLEN, fp);
    
    if ( sscanf(line, "#Detection %s %d %s %d", &observer, &observerSet,
		&detected, &detectedSet)  == 4){

      int numberOfObserver = numberOfRobot( observer, numberOfRobots);
      int numberOfDetected = numberOfRobot( detected, numberOfRobots);
      
      fprintf(stderr, "#Detection %s %d (%d) %s %d (%d)\n",
	      observer, observerSet, numberOfObserver,
	      detected, detectedSet, numberOfDetected);

      /* Found both robots. */
      if ( numberOfObserver >=0 && numberOfDetected >= 0) {
	hasSeen[numberOfObserver][observerSet] = numberOfDetected;
      }
    }	  
  }
  for ( robot = 0; robot < numberOfRobots; robot++)
    for (set = 0; set < MAX_NUMBER_OF_SETS_PER_ROBOT; set++) {
      if ( hasSeen[robot][set] >= 0)
	fprintf(stderr, "#SAW %d %d in %d\n", robot, set, hasSeen[robot][set]);
    }
}



/* Reads in all sample sets and stores the points in the global structures. */
static void
readSamples( int numberOfRobot, int firstSetNumber, int lastSetNumber)
{
  static int robNumber = 0;
  int setNumber = 0;
  FILE *ifp;
  char fileName[255];

  for ( setNumber = firstSetNumber; setNumber <= lastSetNumber; setNumber++) {

    if ( firstSetNumber < lastSetNumber)
      sprintf( fileName, "%s.%d", robotName[numberOfRobot], setNumber);
    else
      sprintf( fileName, "%s", robotName[numberOfRobot]);

    unzip( fileName);
    
    /* Scan for min and max values. */
    if ((ifp = fopen( fileName, "r")) == NULL) {
      fprintf(stderr,"ERROR: Could not open sample file '%s'!\n",
	      fileName);

      samplePoints[robNumber][setNumber] = NULL;
    }
    else {
      
      float posX, posY;
      float dummy;
      float minX = 100000, maxX = -10000;
      float minY = 100000, maxY = -10000;
      float weight;
      int numberOfSamples = 0;
      int dimension = 0;
      struct timeval setTime;
      
      char line[BUFFLEN];
      int numberOfReadSamples = 0;
      int numberOfAcceptedSamples = 0;

      numberOfSamplePoints[robNumber][setNumber] = 0;

      fprintf(stderr, "------------------------ %s ------------------------------\n",
	      fileName);
      
      fgets(line, BUFFLEN, ifp);
      if ( strncmp(line, NUMBER_KEY_WORD, strlen(NUMBER_KEY_WORD)) == 0) {
	unsigned int markLength = strlen( NUMBER_KEY_WORD);
	sscanf( &line[markLength], "%d", &numberOfSamples);
	fprintf(stderr, "File should contain %d samples.\n", numberOfSamples);
      }
      else
	fprintf(stderr, "Keyword %s exected.\n", NUMBER_KEY_WORD);
      
      if (numberOfSamples > MAX_NUMBER_OF_SAMPLES)
	numberOfSamples = MAX_NUMBER_OF_SAMPLES;

      samplePoints[robNumber][setNumber] = (point*) malloc (numberOfSamples * sizeof(point));
      
      fgets(line, BUFFLEN,ifp);
      if ( strncmp(line, DIMENSION_KEY_WORD, strlen(DIMENSION_KEY_WORD)) == 0) {
	unsigned int markLength = strlen( DIMENSION_KEY_WORD);
	sscanf( &line[markLength], "%d", &dimension);
	fprintf(stderr, "Samples have dimension %d.\n", dimension);
	if ( dimension != 3) {
	  fprintf( stderr, "Cannot read samples with dimension not equal 3.\n");
	  exit(0);
	}
      }
      else
	fprintf(stderr, "Keyword %s exected.\n", DIMENSION_KEY_WORD);
      
      fgets(line, BUFFLEN,ifp);
      if ( strncmp(line, MIN_KEY_WORD, strlen(MIN_KEY_WORD)) == 0) {
	unsigned int markLength = strlen( MIN_KEY_WORD);
	sscanf( &line[markLength], "%f %f %f", &minX, &minY, &dummy);
      }
      else
	fprintf(stderr, "Keyword %s exected.\n", MIN_KEY_WORD);
      
      fgets(line, BUFFLEN,ifp);
      if ( strncmp(line, MAX_KEY_WORD, strlen(MAX_KEY_WORD)) == 0) {
	unsigned int markLength = strlen( MAX_KEY_WORD);
	sscanf( &line[markLength], "%f %f %f", &maxX, &maxY, &dummy);
      }
      else
	fprintf(stderr, "Keyword %s exected.\n", MAX_KEY_WORD);
      
      fgets(line, BUFFLEN,ifp);
      if ( strncmp(line, REF_POS_KEY_WORD, strlen(REF_POS_KEY_WORD)) == 0) {
	unsigned int markLength = strlen( REF_POS_KEY_WORD);
	sscanf( &line[markLength], "%f %f %f",
		&(sampleRobotPositions[robNumber][setNumber].x),
		&(sampleRobotPositions[robNumber][setNumber].y),
		&(sampleRobotOrientations[robNumber][setNumber]));
      }
      else
	fprintf(stderr, "Keyword %s exected.\n", REF_POS_KEY_WORD);
      
      fgets(line, BUFFLEN,ifp);
      if ( strncmp(line, TIME_STAMP_KEY_WORD, strlen(TIME_STAMP_KEY_WORD)) == 0) {
	unsigned int markLength = strlen( TIME_STAMP_KEY_WORD);
	sscanf( &line[markLength], "%ld %ld",
		&(sampleTimeStamps[robNumber][setNumber].tv_sec),
		&(sampleTimeStamps[robNumber][setNumber].tv_usec));
      }
      else
	fprintf(stderr, "Keyword %s exected.\n", REF_POS_KEY_WORD);
      
      fprintf(stderr, "Borders of sample set: %f - %f  %f - %f\n", minX, maxX, minY, maxY);
      fprintf(stderr, "Robot Position: %f %f %f\n",
	      sampleRobotPositions[robNumber][setNumber].x,
	      sampleRobotPositions[robNumber][setNumber].y,
	      sampleRobotOrientations[robNumber][setNumber]);
      
      /* Dump the area of the samples. */
      if ( ! minBorderSet || ! maxBorderSet) {
	if ( ! minBorderSet) {
	  minXToDump = minX;
	  minYToDump = minY;
	  minBorderSet = TRUE;
	}
	if ( ! maxBorderSet) {
	  maxXToDump = maxX;
	  maxYToDump = maxY;
	  maxBorderSet = TRUE;
	}
	fprintf( stderr, "# Borders: %f - %f   %f - %f\n",
		 minXToDump, maxXToDump,
		 minYToDump, maxYToDump);
      }
      
      fprintf( stderr, "Start reading samples ... ");
      
      while (!feof(ifp) && numberOfReadSamples < numberOfSamples) {
	
	fgets(line, BUFFLEN,ifp);
	
	/* This should be a sample. */
	if ( sscanf(line, "%f %f %f %f", &posX, &posY, &dummy, &weight)  == 4){
	  numberOfReadSamples++;
	  
	  if ( inBorders( posX, posY)) {
	    samplePoints[robNumber][setNumber][numberOfAcceptedSamples].x = posX;
	    samplePoints[robNumber][setNumber][numberOfAcceptedSamples].y = posY;
	    numberOfAcceptedSamples++;
	  }
	}
	else
	  fprintf( stderr, "Error reading line %s", line);
      }
      fclose(ifp);
      zip( fileName);
      numberOfSamplePoints[robNumber][setNumber] = numberOfAcceptedSamples;
    }
  }
  robNumber++;
}

static void
dumpTree( char* fileName, float scaleFactor, int wireFrame, int logarithmic,
	  FILE* ofp)
{
  if ( fileName) {
  
    float dummy;
    float minX = 100000, maxX = -10000;
    float minY = 100000, maxY = -10000;
    float minDensity = 0.0;
    int dimension = 0;

    float density, maxDensity;
    
    FILE *ifp;

    char line[BUFFLEN];
    
    /* Scan for min and max values. */
    if ((ifp = fopen( fileName, "r")) == NULL) {
      fprintf(stderr,"ERROR: Could not open tree file '%s'!\n",
	      fileName);
      exit(0);
    }
  
    fgets(line, BUFFLEN,ifp);
    if ( strncmp(line, DIMENSION_KEY_WORD, strlen(DIMENSION_KEY_WORD)) == 0) {
      unsigned int markLength = strlen( DIMENSION_KEY_WORD);
      sscanf( &line[markLength], "%d", &dimension);
      fprintf(stderr, "Tree has dimension %d.\n", dimension);
      if ( dimension != 3) {
	fprintf( stderr, "Cannot read tree with dimension not equal 3.\n");
	exit(0);
      }
    }
    else
      fprintf(stderr, "Keyword %s exected.\n", DIMENSION_KEY_WORD);
    
    fgets(line, BUFFLEN,ifp);
    if ( strncmp(line, MIN_KEY_WORD, strlen(MIN_KEY_WORD)) == 0) {
      unsigned int markLength = strlen( MIN_KEY_WORD);
      sscanf( &line[markLength], "%f %f %f", &minX, &minY, &dummy);
    }
    else
      fprintf(stderr, "Keyword %s exected.\n", MIN_KEY_WORD);
  
    fgets(line, BUFFLEN,ifp);
    if ( strncmp(line, MAX_KEY_WORD, strlen(MAX_KEY_WORD)) == 0) {
      unsigned int markLength = strlen( MAX_KEY_WORD);
      sscanf( &line[markLength], "%f %f %f", &maxX, &maxY, &dummy);
    }
    else
      fprintf(stderr, "Keyword %s exected.\n", MAX_KEY_WORD);

    fgets(line, BUFFLEN,ifp);
    if ( strncmp(line, DENSITY_KEY_WORD, strlen(DENSITY_KEY_WORD)) == 0) {
      unsigned int markLength = strlen( DENSITY_KEY_WORD);
      sscanf( &line[markLength], "%f", &maxDensity);
    }
    else
      fprintf(stderr, "Keyword %s exected.\n", DENSITY_KEY_WORD);

    /* Dump the area of the samples. */
    if ( ! minBorderSet || !maxBorderSet) {
      if ( ! minBorderSet) {
	minXToDump = minX;
	minYToDump = minY;
	minBorderSet = TRUE;
      }
      if ( ! maxBorderSet) {
	maxXToDump = maxX;
	maxYToDump = maxY;
	maxBorderSet = TRUE;
      }
      fprintf( stderr, "# Borders: %f - %f   %f - %f\n",
	       minXToDump, maxXToDump,
	       minYToDump, maxYToDump);
    }

    plotXfigHeader( scaleFactor, ofp);
    
    fprintf(stderr, "# Borders of tree: %f - %f  %f - %f\n", minX, maxX, minY, maxY);
    fprintf(stderr, "# Max Density: %f\n", maxDensity);

    fprintf( stderr, "Start reading tree ... ");
    
    if ( logarithmic)
      maxDensity = log( maxDensity);
    
    while (!feof(ifp)) {
    
      fgets(line, BUFFLEN,ifp);

      /* This should be the borders. */
      if ( sscanf(line, "%f %f %f %f %f %f %f",
		  &minX, &minY, &dummy, &maxX, &maxY, &dummy, &density)  == 7){
      
	  if ( inBorders( minX + 1, minY + 1) && inBorders( maxX - 1, maxY - 1)) {

	    if ( wireFrame) {
	      int fillColor = -1;
	      int color = blackAndWhite ? XFIG_BLACK : XFIG_BLUE;
	      plotXfigRectangle( minX, minY,
				 maxX, maxY,
				 color,
				 fillColor, scaleFactor, TREE_DEPTH, ofp);
	    }
	    else {
	      
	      int intensity = xfigGrey( density, minDensity, maxDensity, densityDarkness);

	      if ( intensity > START_GREYSCALE)     
		
		plotXfigRectangle( minX, minY,
				   maxX, maxY,
				   XFIG_BLUE,
				   intensity, scaleFactor, DENSITY_DEPTH, ofp);
	    }
	  }
	  else {
	    fprintf(stderr, "%f %f %f %f\n", minX, minXToDump, maxX, maxXToDump);
	  }
      }
      else
	fprintf( stderr, "Error reading line %s", line);
    }
  
    fprintf( stderr, "done.\n");
  }
}

#define MAX_POSITION_NUMBER 100000
#define MAX_POSITION_NUMBER 100000

int
main( int argc, char *argv[] )
{
  FILE *ifp;
  FILE* ofp = stdout;
  
  char line[BUFFLEN];
  int markLength;
  char *positionLog = NULL, *markerFileName = NULL;
  char* treeFile = NULL, *mapFile = NULL;
  int numberOfObjects = 0;
  int i,s;
  int clip = 0;
  int wireFrame = 0;
  int cellsToBeMerged = 1;
  float thresh = -1;
  int firstSetNumber = 0;
  int lastSetNumber = 150;
  
  int numberOfRobots = 0;
  char* detectionFile = NULL;
  
  
  probabilityGrid map;
  
  float scaleFactor = 4.5;
   
  if (argc < 2)
    {
      fprintf(stderr, "usage: %s [-map m] [-scale s] [-merge numberOfCellsToBeMerged] [-frame]  [-clip threshold] [-pos positionlog] [-depth d] [-marker markerfile] [-min x y] [-max x y] [-samples sampleSet] [-multiSamples sets ] [-tree tree] [-detections detectionFile] \n", argv[0]);
      exit(0);
    }      
   
  for (i=1; i<argc; i++) {

    if ((strcmp(argv[i],"-map")==0)) {
      if ( i < argc - 1)
	mapFile = argv[++i];
      else {
	fprintf( stderr, "ERROR: mapFile must follow keyword -map.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-pos")==0)) {
      if ( i < argc - 1)
	positionLog = argv[++i];
      else {
	fprintf( stderr, "ERROR: logFile must follow keyword -pos.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-scale")==0) {
      if ( i < argc - 1) {
	scaleFactor = atof(argv[++i]);
	fprintf( stderr, "# Scale: %f\n", scaleFactor);
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword -scale.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-clip")==0) {
      clip = 1;
      if ( i < argc - 1) {
	thresh = atof(argv[++i]);
	fprintf( stderr, "# Clip threshold: %f\n", thresh);
	thresh = 1-thresh;
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword -clip.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-mapdarkness")==0) {
      if ( i < argc - 1) {
	mapDarkness = atof(argv[++i]);
	if ( mapDarkness > MAX_GREY_SCALE) {
	  fprintf( stderr, "ERROR: mapDarkness must be in [0:%d].\n",
		   MAX_GREY_SCALE);
	  exit(0);
	}
	fprintf( stderr, "# Mapdarkness: %f\n", mapDarkness);
      }
      else {
	fprintf( stderr, "ERROR: int <0:255> must follow keyword -mapdarkness.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-depth")==0) {
      if ( i < argc - 1) {
	depth = atoi(argv[++i]);
	fprintf( stderr, "# Depth: %d\n", depth);
      }
      else {
	fprintf( stderr, "ERROR: integer must follow keyword -depth.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-min")==0) {
      if ( i < argc - 2) {
	minXToDump = atof(argv[++i]);
	minYToDump = atof(argv[++i]);
	minBorderSet = 1;
	fprintf( stderr, "# Min: %f %f\n", minXToDump, minYToDump);
      }
      else {
	fprintf( stderr, "ERROR: 2 values must follow keyword -min.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-max")==0) {
      if ( i < argc - 2) {
	maxXToDump = atof(argv[++i]);
	maxYToDump = atof(argv[++i]);
	fprintf( stderr, "# Max: %f %f\n", maxXToDump, maxYToDump);
	maxBorderSet = 1;
      }
      else {
	fprintf( stderr, "ERROR: 2 values must follow keyword -max.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-samples")==0)) {
      if ( i < argc - 1) {
	robotName[0] = argv[++i];
	numberOfRobots = 1;
	firstSetNumber = lastSetNumber = 0;
	fprintf( stderr, "# ---- Sample file: %s ----\n", robotName[0]);
      }
      else {
	fprintf( stderr, "ERROR: sampleFile must follow keyword -samples.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-detections")==0)) {
      if ( i < argc - 1) {
	detectionFile = argv[++i];
	fprintf( stderr, "# ---- Detection file: %s ----\n", detectionFile);
      }
      else {
	fprintf( stderr, "ERROR: detectionFile must follow keyword -detection.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-merge")==0)) {
      if ( i < argc - 1) {
	cellsToBeMerged = atoi(argv[++i]);
	fprintf( stderr, "# Merge %d cells.\n", cellsToBeMerged);
      }
      else {
	fprintf( stderr, "ERROR: number must follow keyword -merge.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-tree")==0)) {
      if ( i < argc - 1) {
	treeFile = argv[++i];
	fprintf( stderr, "# ---- Tree file:   %s ----\n", treeFile);
      }
      else {
	fprintf( stderr, "ERROR: treeFile must follow keyword -tree.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-multiSamples")==0)) {
      /* All remaining arguments are assumed to be sample sets. */
      for ( numberOfRobots = 0; i < argc - 1; numberOfRobots++) {
	if (numberOfRobots >= MAX_NUMBER_OF_ROBOTS) {
	  fprintf( stderr, "Too many robots %d >= %d.\n",
		   numberOfRobots, MAX_NUMBER_OF_ROBOTS);
	  exit(0);
	}
	robotName[numberOfRobots] = argv[++i];
	printf( "# ---- Robot no %d: %s ----\n", numberOfRobots,robotName[numberOfRobots]);
      }
      if ( numberOfRobots == 0) {
	fprintf( stderr, "ERROR: at least one sampleFile must follow keyword -multiSamples.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-marker")==0) {
      if ( i < argc - 1)
	markerFileName = argv[++i];
      else {
	fprintf( stderr, "ERROR: markerFile must follow keyword -marker.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-frame")==0) {
      drawFrame = 1;
      fprintf( stderr, "# Draw frame.\n");
    }
    else if (strcmp(argv[i],"-wireFrame")==0) {
      wireFrame = 1;
      fprintf( stderr, "# Draw tree as wire frame.\n");
    }
    else if (strcmp(argv[i],"-bw")==0) {
      blackAndWhite = 1;
      fprintf( stderr, "# Black and white colors.\n");
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[i]);
      getchar();
    }
  }
    
  numberOfObjects = markLength = 0;

  /* Set the global min darkness. */
  globalMinDarkness = NUMBER_OF_GREYSCALES;
  if ( mapFile)
    globalMinDarkness = mapDarkness;
  if ( treeFile && ! wireFrame) {
    globalMinDarkness = 0;
    densityDarkness = 0;
  }

  /*-----------------------------------------------------
   * Read and dump the map.
   *-----------------------------------------------------*/
  if ( mapFile != NULL) {
    if (readProbabilityMap( mapFile, &map)) {
      adjustDumpArea( &map);
      if (numberOfRobots < 1) dumpMap( &map, scaleFactor, clip, thresh, cellsToBeMerged, ofp, 0);
    }
  }
  
  readDetections( detectionFile, numberOfRobots);
  
  /*-----------------------------------------------------
   * Read and dump the samples.
   *-----------------------------------------------------*/
  for ( s = 0; s < numberOfRobots; s++) {
    int dumpBorder = FALSE;
    if ( s == 0)
      dumpBorder = mapFile == NULL;
    readSamples( s, firstSetNumber, lastSetNumber);
  }

  dumpSamples( numberOfRobots, scaleFactor, &map,
	       clip, thresh, cellsToBeMerged);
  
  /*-----------------------------------------------------
   * Read and dump the path.
   *-----------------------------------------------------*/
  if (positionLog != NULL){
    int count, numberOfPositions, skipCount = 0;
    float posX[MAX_POSITION_NUMBER],posY[MAX_POSITION_NUMBER];
    float time;
    if ((ifp = fopen( positionLog,"r")) == NULL) {
      fprintf(stderr,"ERROR: Could not open position file '%s'!\n",
	      positionLog);
      exit(0);
    }
    

    numberOfPositions = 0;
    while (!feof(ifp) && numberOfPositions < MAX_POSITION_NUMBER){
      float tmp1, tmp2, tmp3;
      fgets(line,BUFFLEN,ifp);
      if ((sscanf(line, "position: %f %f", &posX[numberOfPositions],
		 &posY[numberOfPositions]) == 2) &&
	  inBorders( posX[numberOfPositions], posY[numberOfPositions])) {
	if (skipCount % SKIP == 0)
	  numberOfPositions++;
	skipCount++;
      }
      else if (sscanf(line, "%f %f %f %f %f %f", &time, &posX[numberOfPositions],
		 &posY[numberOfPositions], &tmp1, &tmp2, &tmp3) == 6){
	if ((skipCount % SKIP == 0) && (inBorders( posX[numberOfPositions],
						   posY[numberOfPositions])))
	  numberOfPositions++;
	skipCount++;
      }
      else if (sscanf(line, "%f %f", &posX[numberOfPositions],
		 &posY[numberOfPositions]) == 2){
	if ((skipCount % SKIP == 0) && (inBorders( posX[numberOfPositions],
						   posY[numberOfPositions])))
	  numberOfPositions++;
	skipCount++;
      }
      else if (sscanf(line, "#ROBOT %f %f", &posX[numberOfPositions],
		      &posY[numberOfPositions]) == 2){
	if ((skipCount % SKIP == 0) && (inBorders( posX[numberOfPositions],
						   posY[numberOfPositions])))
	  numberOfPositions++;
	skipCount++;
      }
    }
    
    if (numberOfPositions >= MAX_POSITION_NUMBER)
      fprintf(stderr, "Warning too many positions in %s\n", positionLog);
    
    if (numberOfPositions > 0){
      fprintf( ofp, "%s %d\n      ",
	       "2 1 0 1 -1 7 0 0 -1 0.000 0 0 -1 0 0",
	       numberOfPositions);
      
      for (count = 0; count < numberOfPositions; count++)
	fprintf( ofp, "%d %d ",
		 round((posX[count] - minXToDump) * scaleFactor),
		 round((maxYToDump - posY[count]) * scaleFactor));
      
      fprintf(ofp, "\n");

      {

	int intPosX = round((posX[0] - minXToDump) * scaleFactor);
	int intPosY = round((maxYToDump - posY[0]) * scaleFactor);
	int radius = (int) (30 * scaleFactor);

	if (radius < 20) radius = 20;
	
	fprintf( ofp, "1 3 0 1 -1 7 0 0 -1 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		 intPosX, intPosY, radius, radius, intPosX, intPosY,
		 intPosX + radius, intPosY + radius);
	
	intPosX = round((posX[numberOfPositions - 1 ] - minXToDump) * scaleFactor);
	intPosY = round((maxYToDump - posY[numberOfPositions -1 ]) * scaleFactor);
	radius = radius / 4;
	if (radius < 20) radius = 20;
	
	
	fprintf( ofp, "1 3 0 1 -1 0 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		 intPosX, intPosY, radius, radius, intPosX, intPosY,
		 intPosX + radius, intPosY + radius);
	
      }
    }
    
    fclose(ifp);
  }

   
  /*-----------------------------------------------------
   * Read and dump the marked positions.
   *-----------------------------------------------------*/
  if (markerFileName != NULL){
    if ((ifp = fopen( markerFileName,"r")) == NULL) {
      fprintf(stderr,"ERROR: Could not marker file '%s'!\n", markerFileName);
    }
    else {
      while (fgets(line,BUFFLEN,ifp) != NULL){
	char marker[BUFFLEN];
	float posX, posY;
	int intPosX, intPosY, radius;
	if ( (sscanf( line, "%s %f %f", marker, &posX, &posY) == 3) &&
	     ( inBorders( posX, posY))) {
	  intPosX = round((posX - minXToDump) * scaleFactor);
	  intPosY = round((maxYToDump - posY) * scaleFactor);
	  radius = 20;
	  fprintf( ofp, "1 3 0 1 -1 0 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		   intPosX, intPosY, radius, radius, intPosX, intPosY,
		   intPosX + radius, intPosY + radius);
	  fprintf( ofp, "4 0 -1 0 0 0 24 0.0000 4 255 255 %d %d %s\\001\n",
		   intPosX + 5*radius,
		   intPosY + 5*radius,
		   marker);
	  

	}
      }
      
      fclose(ifp);
    }
  }

  dumpTree( treeFile, scaleFactor, wireFrame, FALSE, ofp);
  if ( whirlFile) {
    fprintf( whirlFile, "\n");
    fclose( whirlFile);
  }
  exit(0);
}







