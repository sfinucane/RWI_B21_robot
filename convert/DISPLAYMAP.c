
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/DISPLAYMAP.c,v $
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
 * $Log: DISPLAYMAP.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.8  2000/03/18 15:26:46  wolfram
 * Added option for line style
 *
 * Revision 1.7  1999/03/22 21:49:33  wolfram
 * Files now compile again.
 *
 * Revision 1.6  1999/01/19 05:00:05  wolfram
 * Fixed a bug
 *
 * Revision 1.5  1998/10/19 19:41:25  fox
 * *** empty log message ***
 *
 * Revision 1.4  1998/02/24 08:21:30  wolfram
 * Added the -scale option
 *
 * Revision 1.3  1998/02/21 15:18:57  wolfram
 * DISPLAYMAP now adopts the scale according to the size of the map.
 * SCAN2MAP now has several options. Scan files are
 * given as command line arguments.
 *
 * Revision 1.2  1997/03/11 08:55:33  wolfram
 * Small changes
 *
 * Revision 1.1  1997/02/07 16:38:18  wolfram
 * Small program for displaying grid maps
 *
 * Revision 1.1.1.1  1996/09/22 16:47:00  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include "EZX11.h"

#define MAX_STRING_LENGTH 256
#define BUFFLEN 1024
#define SKIP 1

#define MYMAXCOLORS     155
#define COL_BACK        0                  /* background of window       */
#define COL_UNKNOWN     C_LAWNGREEN           /* color of unknown area      */
#define COL_RED         C_RED
#define COL_BLACK       54
#define COL_WHITE       154
#define COL_ROBOT       C_YELLOW
#define FALSE 0
#define TRUE 1

int
round(float x)
{
  return (int) (x + 0.5);
}

static int
fieldColor(float f){
  int color;
  if (f < 0.0)
    color = COL_UNKNOWN;
  else
    color = COL_WHITE - (int) round((1.0 - f) *(COL_WHITE-COL_BLACK));
  return color;
}


float
fMin(float x, float y)
{
  if (x < y)
    return(x);
  else
    return(y);
}

float
fMax(float x, float y)
{
  if (x > y)
    return(x);
  else
    return(y);
}

typedef int bool;
typedef float mapProbability;
typedef float probability;

/* This struct contains the given position / occupancy probabilities.
 */
typedef struct {
  bool initialized;
  int sizeX;
  int sizeY;
  int origsizeX;
  int origsizeY;
  float offsetX;
  float offsetY;
  int resolution;
  mapProbability average;
  mapProbability unknown;
  mapProbability** prob;
} probabilityGrid;

typedef struct {
    float x;
    float y;
    float rot;
} realPosition;


typedef struct {
  EZXW_p window;
  int startX;
  int startY;
  int sizeX;
  int sizeY;
  int gridSizeX;
  int gridSizeY;
  int gridOffsetX;
  int gridOffsetY;
  int gridResolution;
  int scale;
  int planeType;
  bool firstTimePosition;
  realPosition pos;
}  gridWindow;

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

  m->offsetX += m->resolution * min_x;
  m->offsetY += m->resolution * min_y;

  fprintf(stderr,"# Map shrinked: %d %d  offset: (%f, %f)\n",m->sizeX,m->sizeY,
	  m->offsetX, m->offsetY);

  for(x=0; x<m->sizeX; x++)
    for(y=0; y<m->sizeY; y++)
      m->prob[x][y] = m->prob[x+min_x][y+min_y];

  return(1);
  
}



bool
readGridMap(char *mapName, char *extension, probabilityGrid *m){
   int x,y;
   float temp;
   char line[MAX_STRING_LENGTH], fileName[MAX_STRING_LENGTH];
   FILE *fp;

   sprintf(fileName, "%s%s", mapName, extension);

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

   setStatistics(m);
   fprintf(stderr, "# done\n");
   return TRUE;
   
}



static int
gridWindowScale(int sizeX, int sizeY, int minScale) {
    int scaleX = (int) (1024 / sizeX);
    int scaleY = (int) (768 / sizeY);

    if (scaleX < minScale)
	scaleX = minScale;
    if (scaleY < minScale)
        scaleY = minScale;

    if (scaleY < scaleX)
	return(scaleY);
    else
	return(scaleX);
}

gridWindow *
createMapWindow(probabilityGrid *m, char* text, int x, int y, int minScale) {
  /* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */

    char corner[80];
    gridWindow *mapwin;
    
    mapwin = (gridWindow *) malloc(sizeof(gridWindow));
    mapwin->scale = gridWindowScale(m->sizeX, m->sizeY, minScale);

    mapwin->gridSizeX = m->sizeX;
    mapwin->gridSizeY = m->sizeY;
    mapwin->gridResolution = m->resolution;
    mapwin->gridOffsetX = round(m->offsetX / m->resolution);
    mapwin->gridOffsetY = round(m->offsetY / m->resolution);
    
    mapwin->sizeX = m->sizeX * mapwin->scale;
    mapwin->sizeY = m->sizeY * mapwin->scale;

    mapwin->firstTimePosition = TRUE;

    sprintf(corner,"+%d+%d",x,y);
    
    EZX_NoMotionEvents();
    mapwin->window = EZX_MakeWindow(text,mapwin->sizeX,mapwin->sizeY,corner);
    EZX_SetWindowBackground (mapwin->window,C_RED);
    EZX_Flush();

    mapwin->startX = 0;
    mapwin->startY = 0;
    return(mapwin);
}

int
coordinateInMap(int x, int y, probabilityGrid m) {
    return (x >= 0 && x < m.sizeX && y >= 0 && y < m.sizeY); 
}


mapProbability
occupancyProbabilityMap(int x, int y, probabilityGrid m) {
    if (coordinateInMap(x, y, m))
	if (m.prob[x][y] != m.unknown)
	    return m.prob[x][y];
	else
	    return m.average;
    else
	return m.average;
}



void
displayMapWindow(probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

    int x,y;
    
    int winX, winY;
    int endX = mapwin->startX + m->sizeX * mapwin->scale;
    int endY = mapwin->startY + m->sizeY * mapwin->scale;

/*     EZX_SetColor(COL_UNKNOWN); */
/*     EZX_FillRectangle(mapwin->window,mapwin->startX,mapwin->startY,endX,endY); */
    
    y=m->sizeY;
    for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
	y--;
	x=0;
	for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	  EZX_SetColor(fieldColor( occupancyProbabilityMap(x,y,*m)));
	  EZX_FillRectangle(mapwin->window,winX,winY,
			    mapwin->scale,mapwin->scale);
	  x++;
	}
      }

    EZX_Flush();

}


void
clearMapWindow(gridWindow *win)
{
  EZX_SetColor(C_WHITE);
  EZX_FillRectangle(win->window, 0, 0, win->sizeX, win->sizeY);
  EZX_Flush();
}







int
main(int argc, char* argv[]) 
{

  FILE *ifp;

  gridWindow *mapWin;
  probabilityGrid map;
  char* mapFileName;
  char *positionLog = NULL;
  int scale = 0, i;
  int sizeOfPositions = 0;
    
  if (argc < 2)
    {
      fprintf(stderr, "usage: %s mapFile [-scale scale] [-pos positionlog] [-psize sizeOfPositions]\n",
	      argv[0]);
      exit(0);
    }      
   
  mapFileName = argv[1];
  if ((ifp = fopen( mapFileName,"r")) == NULL) {
    fprintf(stderr,"ERROR: Could not open map file '%s'!\n",mapFileName);
    exit(0);
  }

  for (i=2; i<argc; i++) {
    if ((strcmp(argv[i],"-pos")==0)) {
      if ( i < argc - 1) 
	positionLog = argv[++i];
      else {
	fprintf( stderr, "ERROR: positionFile must follow keyword -pos.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-psize")==0)) {
      if ( i < argc - 1) 
	sizeOfPositions = atoi(argv[++i]);
      else {
	fprintf( stderr, "ERROR: size must follow keyword -psize.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-scale")==0) {
      if ( i < argc - 1) {
	scale = atoi(argv[++i]);
	fprintf( stderr, "Scale: %d\n", scale);
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword -scale.\n");
	exit;
      }
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[i]);
      getchar();
    }
  }
  
  if (readGridMap(mapFileName, "", &map)){
    shrinkMap( &map);
    mapWin = createMapWindow(&map, "map", 0, 0, scale);
    displayMapWindow(&map, mapWin);

    /* Read the path of the robot. */
    if (positionLog != NULL){

#define MAX_POSITION_NUMBER 10000

      char line[BUFFLEN];
      int count, numberOfPositions, skipCount = 0;
      float posX[MAX_POSITION_NUMBER],posY[MAX_POSITION_NUMBER];
      float time;
      int mapSizeY = map.sizeY * map.resolution;
      
      if ((ifp = fopen( positionLog,"r")) == NULL) {
	fprintf(stderr,"ERROR: Could not open position file '%s'!\n",
		positionLog);
	exit(0);
      }
      
      fprintf(stderr, "# Get positions from %s.\n", positionLog);
      
      numberOfPositions = 0;
      while (!feof(ifp) && numberOfPositions < MAX_POSITION_NUMBER){
	fgets(line,BUFFLEN,ifp);
	if (sscanf(line, "%f %f %f", &time, &posX[numberOfPositions],
		   &posY[numberOfPositions]) == 3){
	  if (skipCount % SKIP == 0)
	    numberOfPositions++;
	  skipCount++;
	}
	else if (sscanf(line, "#ROBOT %f %f", &posX[numberOfPositions],
			&posY[numberOfPositions]) == 2){
	  if (skipCount % SKIP == 0)
	    numberOfPositions++;
	  skipCount++;
	}
      }
      
      if (numberOfPositions >= MAX_POSITION_NUMBER)
	fprintf(stderr, "Warning to many positions in %s\n", positionLog);
      
      /* Plot the path as a polyline. */
      if (numberOfPositions > 0){

	float scaleForRealPositions = (float) mapWin->scale / (float) map.resolution;
	
	EZX_SetColor( C_RED);
	for (count = 0; count < numberOfPositions; count++) {
	  EZX_FillCircle( mapWin->window,
			  scaleForRealPositions * posX[count],
			  scaleForRealPositions * (mapSizeY - posY[count]),
			  sizeOfPositions * scaleForRealPositions);
	  if ( count > 0)
	    EZX_DrawLine( mapWin->window,
			  scaleForRealPositions * posX[count-1],
			  scaleForRealPositions * (mapSizeY - posY[count-1]),
			  scaleForRealPositions * posX[count],
			  scaleForRealPositions * (mapSizeY - posY[count]));
	}
      }
      fclose(ifp);
    }
    EZX_Flush();
    if (0) {
      EZX_EventPtr event;
      EZX_GetEvent ( event);

    }
    getchar();
  }
  
  exit(0);
}

  
  


  
