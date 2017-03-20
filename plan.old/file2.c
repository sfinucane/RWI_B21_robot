
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/plan/file2.c,v $
 *****
 ***** Created by:      $Author: fox $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 1999/04/18 17:18:30 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: file2.c,v $
 * Revision 1.1  1999/04/18 17:18:30  fox
 * Just an additional file.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "tcx.h"
#define TCX_define_variables /* this makes sure variables are installed */


#include "PLAN-messages.h"
#include "COLLI-messages.h"
#include "MAP-messages.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"


#include "PLAN.h"
#include "o-graphics.h"

#include <bUtils.h>

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/* This struct contains the given position / occupancy probabilities.
 */
#define MAX_STRING_LENGTH 256
#define MAP_PROBABILITY       5
#define MINIMUM_MAPPROBABILITY ((mapProbability) 0.00001)
#define MAXIMUM_MAPPROBABILITY ((mapProbability) 0.99999)
FILE *logFile = NULL;
#include <stdarg.h>
int
writeLog( char* fmt, ...)
{
 va_list                args;
 int			bytes;

 if ( logFile != NULL) {
   
   va_start(args, fmt);
   bytes = vfprintf(logFile, fmt, args);
   va_end(args);
   
/* #define IMMEDIATE_WRITE  */
#ifdef IMMEDIATE_WRITE
   fflush(logFile);
#endif
   
   return bytes;
 }
 else
   return 0;
}


typedef struct {
  int cadMap;
  float unknown;
  float desiredResolution;
  char mapFileName[MAX_STRING_LENGTH];
} mapParameters;

mapParameters globalMapParameters;



/* The different types possible for allocation. */

#define FLOAT                 0
#define DOUBLE                1
#define INT                   2
#define BOOL                  3
#define PROBABILITY           4
#define MAP_PROBABILITY       5
#define EXPECTED_DISTANCE     6
#define REAL_POSITION         7
#define PIXEL                 8
#define POSITION              9


void**
allocate2D( int dim1, int dim2, int type)
{
  switch (type) {
  case INT:
    {
      int x;
      int** pt;
      if ( (pt = (int**) malloc( dim1 * sizeof( int*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (int*) malloc( dim2 * sizeof(int))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case MAP_PROBABILITY:
    {
      int x;
      mapProbability** pt;
      if ( (pt = (mapProbability**)
	    malloc( dim1 * sizeof( mapProbability*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (mapProbability*)
	malloc( dim2 * sizeof(mapProbability))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case PROBABILITY:
    {
      int x;
      probability** pt;
      if ( (pt = (probability**)
	    malloc( dim1 * sizeof( probability*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (probability*)
	malloc( dim2 * sizeof(probability))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case FLOAT:
    {
      int x;
      float** pt;
      if ( (pt = (float**)
	    malloc( dim1 * sizeof( float*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (float*)
	malloc( dim2 * sizeof(float))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case POSITION:
    {
      int x;
      position** pt;
      if ( (pt = (position**)
	    malloc( dim1 * sizeof( position*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (position*)
	malloc( dim2 * sizeof(position))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case REAL_POSITION:
    {
      int x;
      realPosition** pt;
      if ( (pt = (realPosition**)
	    malloc( dim1 * sizeof( realPosition*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (realPosition*)
	malloc( dim2 * sizeof(realPosition))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case PIXEL:
    {
      int x;
      pixel** pt;
      if ( (pt = (pixel**)
	    malloc( dim1 * sizeof( pixel*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (pixel*)
	malloc( dim2 * sizeof( pixel))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  default:
    fprintf( stderr, "Type %d not implemented.\n", type);
    return NULL;
  }
}

int
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

  if ( logFile != NULL) 
    fprintf(logFile,"# Map shrinked: %d %d\n",m->sizeX,m->sizeY);
  fprintf(stderr,"# Map shrinked: %d %d\n",m->sizeX,m->sizeY);

  m->offsetX = min_x;
  m->offsetY = min_y;

  for(x=0; x<m->sizeX; x++)
    for(y=0; y<m->sizeY; y++)
      m->prob[x][y] = m->prob[x+(int) m->offsetX][y+(int) m->offsetY];

  return(1);
  
}

void
setStatistics( probabilityGrid *m)
{
  int y, x;
  
  int knownFields = 0;
  probability sum = 0.0;

  for (x=0; x<m->sizeX; x++)
    for (y=0; y<m->sizeY; y++)
      if ( m->prob[x][y] != m->unknown) {
	knownFields++;
	sum += m->prob[x][y];
      }
  
  if ( knownFields > 0)
    m->average = sum / knownFields;
  else
    m->average = m->unknown;
}


char
readGridMap(char *mapName, probabilityGrid *m){
   int x,y;
   float temp;
   char line[MAX_STRING_LENGTH], fileName[MAX_STRING_LENGTH];
   FILE *fp;

   sprintf(fileName, "%s", mapName);

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
	   writeLog("# Map resolution: %d cm\n",m->resolution);
	 }
     }
     if (strncmp(line,"robot_specifications->autoshifted_x",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetX)) != 0 ) {
	 m->offsetX = m->offsetX;
	 writeLog("# Map offsetX: %g cm\n",m->offsetX);
       }
     }
     if (strncmp(line,"robot_specifications->autoshifted_y",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetY)) != 0 ) {
	 m->offsetY = m->offsetY;
	 writeLog("# Map offsetY: %g cm\n",m->offsetY);
       }
     }
   }
   if (sscanf (line,"global_map[0]: %d %d",&m->sizeY, &m->sizeX)
       != 2 ) {
     fprintf(stderr,"ERROR: corrupted file %s\n",fileName);
     fclose(fp);
     return FALSE;
   }
   
   writeLog("# Map size: %d %d\n",m->sizeX,m->sizeY);
   
   m->unknown = globalMapParameters.unknown;
   m->offsetX = m->offsetY = 0.0;
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->shiftedX = 0;
   m->shiftedY = 0;
   m->maxRealX = m->offsetX + m->sizeX * m->resolution;
   m->maxRealY = m->offsetY + m->sizeY * m->resolution;
   m->numberOfFreeCells = 0;
   m->freeSpace = NULL;
   
   m->prob = (mapProbability**) allocate2D(m->sizeX,
					   m->sizeY,
					   MAP_PROBABILITY);
   
   if (m->prob == (mapProbability**) NULL){
      fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n", fileName);
      fclose(fp);
      return FALSE;
   }
   
   for (x=0;x<m->sizeX; x++)
      for (y=0;y<m->sizeY; y++) {
	 fscanf(fp,"%e",&temp);
	 if (temp < 0.0 || temp > 1.0)
	   m->prob[x][y] = globalMapParameters.unknown;
	 else {
	   m->prob[x][y] = 1-temp;	   
	   if (m->prob[x][y] < MINIMUM_MAPPROBABILITY)
	     m->prob[x][y] = MINIMUM_MAPPROBABILITY;
	   else if (m->prob[x][y] > MAXIMUM_MAPPROBABILITY)
	     m->prob[x][y] = MAXIMUM_MAPPROBABILITY;
	 }
      }

   shrinkMap(m);
   setStatistics(m);
   fprintf(stderr, "# done\n");
   return TRUE;
}


float 
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

char
sendMapReplyType( probabilityGrid* map,
		  char inverted,
		  char* name, int type)
{
  if ( 1 ) {

    MAP_partial_map_reply_type data;
    int x, y;
    probability min = 1e20, max = -1e20;

    for ( x = 0; x < map->sizeX; x++)
      for ( y = 0; y < map->sizeY; y++)
	if ( map->prob[x][y] != map->unknown) {
	  if (map->prob[x][y] < min)
	    min = map->prob[x][y];
	  if (map->prob[x][y] > max) {
	    max = map->prob[x][y];
	  }
	}

    /* Now copy the values in the tcx structure. */
    data.first_x = 0;
    data.first_y = 0;
    data.delete_previous_map = 0;
    data.number_of_map = type;

    data.resolution = map->resolution;
    data.size_x  = map->sizeX;
    data.size_y  = map->sizeY;

    data.char_values = (unsigned char*)
      malloc( data.size_x * data.size_y * sizeof(unsigned char));

    for ( x = 0; x < data.size_x; x++)
      for ( y = 0; y < data.size_y; y++)
	if ( map->prob[x][y] != map->unknown) {
	  if ( inverted)
	    data.char_values[x * data.size_y + y] =
	      fNorm( map->prob[x][y], min, max, 230, 1);
	  else
	    data.char_values[x * data.size_y + y] =
	      fNorm( map->prob[x][y], min, max, 1, 230);
	}
	else
	  data.char_values[x * data.size_y + y] = 0;

    writeLog( "# Send map to %s.\n", name);

    MAP_partial_map_reply_handler(NULL, &data);

    /* tcxSendMsg(module, "MAP_partial_map_reply", &data); */

    free( data.char_values);
    return TRUE;
  }
  else {
    writeLog( "Error: attempt to send reply map to unconnected
module.\n");
    return FALSE;
  }
}


