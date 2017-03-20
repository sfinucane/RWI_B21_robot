
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/MAP2SIM.c,v $
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
 * $Log: MAP2SIM.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.2  2000/01/26 23:09:41  wolfram
 * Changes ....
 *
 * Revision 1.1  2000/01/26 22:54:40  wolfram
 * Utility to convert .map files into simulator files.  It does a runlength
 * encoding so that the resutling files are moderately small.
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


#define BUFFLEN 1024

#define TRUE 1
#define FALSE 0
#define CUBE 0


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

int depth = 0;

/* The borders of the area to be displayed. */
float minXToDump, minYToDump, maxXToDump, maxYToDump;
int minBorderSet = 0, maxBorderSet = 0;

/* The borders of the defined area. */
float yZeroLine;

static void
dumpFrame(float scaleFactor);

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
       if (temp >=  0.0 && temp <= 1.0){
	   m->prob[x][y] = (float) (1.0 - temp);
       }
       else
	   m->prob[x][y] = m->unknown;
     }
   
   shrinkMap(m);
   fprintf(stderr, "# done\n");

   return TRUE;
}

int
round(float x)
{
  return (int) (x + 0.5);
}



static void
dumpMap( probabilityGrid* map, float thresh, int unknownToObject)
{
  int x, y;
  float maxProb = 0.0;
  float minProb = 1000.0;
  float resolution = map->resolution;
  int count = 0;

  printf("MAP %f %f %f %f\n", 0.0, 0.0, (map->sizeX)*resolution, 
	 (map->sizeY)*resolution);
  
  printf("ROBOT %f %f %f \n", 0.0, 0.0, 0.0);
  
  for ( y = 0; y < map->sizeY;y++) {
    for ( x = 0; x < map->sizeX;){
      if (map->prob[x][y] > thresh || 
	  (unknownToObject && map->prob[x][y] == map->unknown)){
	int lengthX = 1;
	while ((x + lengthX) < map->sizeX && (map->prob[x+lengthX][y] > thresh 
					      || (unknownToObject 
						  && map->prob[x+lengthX][y] 
						  == map->unknown)))
	  lengthX++;
	printf("CUBE %f %f %f %f %f %f 0.0\n",
	       (x + lengthX/2.0) * resolution,
	       (y + 0.5) * resolution,
	       (float) 150.0,
	       lengthX * resolution,
	       resolution,
	       (float) 300.0);
	x = x + lengthX;
	count++;
      }
      else x++;
    }
  }
  fprintf(stderr, "# Wrote %d objects\n", count);
  fflush(stdout);
}


int
main( int argc, char *argv[] )
{
   
  char line[BUFFLEN];
  char *mapFile = NULL;
  float thresh = 0.5;
  probabilityGrid map;
  int i;
  int unknownToObject = 0;


  if (argc < 2)
    {
      fprintf(stderr, "usage: %s -map m [-clip threshold] [-unknownToObject]\n", argv[0]);
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
    else if (strcmp(argv[i],"-clip")==0) {
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
    else if (strcmp(argv[i],"-unknownToObject")==0) {
      unknownToObject = 1;
    }
    else {
      fprintf(stderr, "usage: %s [-map m] [-clip threshold] -invert\n", argv[0]);
      exit(0);
    }
  }
  
  /*-----------------------------------------------------
   * Read and dump the map.
   *-----------------------------------------------------*/
  if ( mapFile != NULL) {
    if (readProbabilityMap( mapFile, &map)) {
      dumpMap( &map, thresh, unknownToObject);
    }
  }

  exit(0);
}






