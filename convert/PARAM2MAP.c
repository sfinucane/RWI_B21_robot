
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/PARAM2MAP.c,v $
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
 * $Log: PARAM2MAP.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1999/03/22 21:49:33  wolfram
 * Files now compile again.
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

#define MAX_NUMBER_OF_OBJECTS 2000
#define ROOM_MARK "room"
#define CORR_MARK "corridor"
#define DOOR_MARK "door"
#define FOYER_MARK "foyer"
#define AREA_MARK "area"
#define OBSTACLE_MARK "obstacle"
#define WALL_MARK "wall"
#define ROOM 0
#define CORRIDOR 1
#define DOOR 2
#define FOYER 3
#define AREA 4
#define OBSTACLE 5
#define WALL 6
#define UNKNOWN_OBJECT -1
#define COL_BLACK       54
#define COL_WHITE       154
#define COL_UNKNOWN     C_RED
#define BUFFLEN 1024

typedef struct{
  float x;
  float y;
} point;

typedef struct{
  point point1;
  point point2;
} rectangle;


typedef struct{
  int type;
  rectangle rect;
} object;


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



int
main(int argc, char **argv)
{
  FILE *fp;

  object obj[MAX_NUMBER_OF_OBJECTS];
  char line[BUFFLEN];
  int type, markLength, cnt, numberOfObjects;
  int x,y;
  float x1, y1, x2, y2;
  float minXRect, maxXRect, minYRect, maxYRect;
  float minX, maxX, minY, maxY;
  int startX, endX, startY, endY;
  int mapSizeX;
  int mapSizeY;
  float **map;
  
  EZXW_p window;
  
  int mapResolution;
  
  if (argc != 3)
  {
    fprintf(stderr, "usage: cmu2map file resolution\n");
    exit(0);
  }      

  mapResolution = atoi(argv[2]);
  if (mapResolution <= 0)
  {
    fprintf(stderr, "Wrong resolution: %d\n", mapResolution);
    exit(0);
  }
  else
    fprintf(stderr, "Map resolution: %d\n", mapResolution);
  
  if ((fp = fopen(argv[1],"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open file '%s'!\n",argv[1]);
    exit(0);
  }
  
  cnt = markLength = 0;
  
  while (fgets(line,BUFFLEN,fp) != NULL){
    if (strncmp(line,ROOM_MARK,strlen(ROOM_MARK)) == 0){
      type = ROOM;
      markLength = strlen(ROOM_MARK);
    }
    else if (strncmp(line,CORR_MARK,strlen(CORR_MARK)) == 0){
      type = CORRIDOR;
      markLength = strlen(CORR_MARK);
    }
    else if (strncmp(line,DOOR_MARK,strlen(DOOR_MARK)) == 0){
      type = DOOR;
      markLength = strlen(DOOR_MARK);
    }
    else if (strncmp(line,FOYER_MARK,strlen(FOYER_MARK)) == 0){
      type = FOYER;
      markLength = strlen(FOYER_MARK);
    }
    else if (strncmp(line,WALL_MARK,strlen(WALL_MARK)) == 0){
      type = WALL;
      markLength = strlen(WALL_MARK);
    }
    else if (strncmp(line,AREA_MARK,strlen(AREA_MARK)) == 0){
      type = AREA;
      markLength = strlen(AREA_MARK);
    }
    else if (strncmp(line,OBSTACLE_MARK,strlen(OBSTACLE_MARK)) == 0){
      type = OBSTACLE;
      markLength = strlen(OBSTACLE_MARK);
    }
    else
      type = UNKNOWN_OBJECT;

    if (type != UNKNOWN_OBJECT){
      if (sscanf(&line[markLength],"%f %f %f %f", &x1, &x2, &y1, &y2) == 4)
      {
/* 	fprintf(stderr, "Line: %s", &line[markLength]); */
/* 	fprintf(stderr, "Scanned: %f %f %f %f\n", x1, y1, x2, y2); */
	if (cnt < MAX_NUMBER_OF_OBJECTS){
	  obj[cnt].type = type;
	  obj[cnt].rect.point1.x = x1;
	  obj[cnt].rect.point1.y = y1;
	  obj[cnt].rect.point2.x = x2;
	  obj[cnt].rect.point2.y = y2;
/* 	  fprintf(stderr, "Object: N=%d, Type=%d, (%f,%f)->(%f,%f)\n", cnt, */
/* 	    obj[cnt].type, obj[cnt].rect.point1.x, obj[cnt].rect.point1.y, */
/* 	    obj[cnt].rect.point2.x, obj[cnt].rect.point2.y); */
	  cnt++;
	}
	else
	{
	  fprintf(stderr, "Too many objects .. abort\n");
	  exit(0);
	}
      }
    }
  }

  fclose(fp);

  numberOfObjects = cnt;
  fprintf(stderr, "Number of Objects found: %d\n", numberOfObjects);

  if (numberOfObjects <= 0)
    exit(0);

  /* find map parametersl */

  minX = fMin(obj[0].rect.point1.x,obj[0].rect.point2.x);
  maxX = fMax(obj[0].rect.point1.x,obj[0].rect.point2.x);
  minY = fMin(obj[0].rect.point1.y,obj[0].rect.point2.y);
  maxY = fMax(obj[0].rect.point1.x,obj[0].rect.point2.y);
  
  for (cnt = 1; cnt < numberOfObjects; cnt++)
  {
    minXRect = fMin(obj[cnt].rect.point1.x,obj[cnt].rect.point2.x);
    maxXRect = fMax(obj[cnt].rect.point1.x,obj[cnt].rect.point2.x);
    minYRect = fMin(obj[cnt].rect.point1.y,obj[cnt].rect.point2.y);
    maxYRect = fMax(obj[cnt].rect.point1.y,obj[cnt].rect.point2.y);

    minX = fMin(minX, minXRect);
    maxX = fMax(maxX, maxXRect);
    minY = fMin(minY, minYRect);
    maxY = fMax(maxY, maxYRect);
  }
  fprintf(stderr, "(%f, %f) --> (%f, %f)\n", minX, maxX, minY, maxY);

  mapSizeX = round((maxX - minX) / mapResolution);
  mapSizeY = round((maxY - minY) / mapResolution);

  fprintf(stderr, "Map size: %d x %d\n", mapSizeX, mapSizeY);
  
  /*  allocating memory */

  if ((map = (float **) malloc(mapSizeX * sizeof(float *))) == NULL){
    fprintf(stderr, "Error allocating map\n");
    exit(0);
  }
  else
    for (x = 0; x < mapSizeX; x++)
      if ((map[x] = (float *) malloc(mapSizeY * sizeof(float))) == NULL){
	fprintf(stderr, "Error allocating map\n");
	exit(0);
      }

  /* filling everything black */

  for (x = 0; x < mapSizeX; x++)
    for (y = 0; y < mapSizeY; y++)
      map[x][y] = 0.0;

  /* painting rooms, doors and corridors */

  for (cnt = 0; cnt < numberOfObjects; cnt++)
    if (obj[cnt].type == ROOM || obj[cnt].type == CORRIDOR ||
	obj[cnt].type == DOOR || obj[cnt].type == FOYER ||
	obj[cnt].type == AREA || obj[cnt].type == OBSTACLE ||
	obj[cnt].type == WALL){
/*       fprintf(stderr, "Object: N=%d, Type=%d, (%f,%f)->(%f,%f)\n", cnt, */
/* 	      obj[cnt].type, obj[cnt].rect.point1.x, obj[cnt].rect.point1.y, */
/* 	      obj[cnt].rect.point2.x, obj[cnt].rect.point2.y); */
      startX = round((obj[cnt].rect.point1.x - minX) / mapResolution);	  
      endX = round((obj[cnt].rect.point2.x - minX) / mapResolution);
      if (endX == startX) endX++;
      startY = round((obj[cnt].rect.point1.y - minY) / mapResolution);
     endY = round((obj[cnt].rect.point2.y - minY) / mapResolution);
       if (endY == startY) endY++;
/*       fprintf(stderr, "(%d, %d) --> (%d, %d)\n", startX,startY, endX, endY); */
      for (x = startX; x < endX; x++)
	for (y = startY; y < endY; y++)
	  if (obj[cnt].type == DOOR || obj[cnt].type == OBSTACLE)
	    map[x][y] = 0.5;
	  else if (obj[cnt].type == WALL)
	    map[x][y]= 0.0;
	  else
	    map[x][y] = 1.0;
    }


  EZX_NoMotionEvents();
  window = EZX_MakeWindow("Map",mapSizeX,mapSizeY,"+0+0");
  EZX_SetWindowBackground (window,C_RED);
  EZX_Flush();

  for (x = 0; x < mapSizeX; x++){
    for (y = 0; y < mapSizeY; y++){
      EZX_SetColor(fieldColor(map[x][y]));
      EZX_FillRectangle(window,x,mapSizeY-y-1,1,1);
    }
  }

  getchar();

  printf("robot_specifications->resolution %d\n", mapResolution);
  printf("global_map[0]: %d %d\n", mapSizeY, mapSizeX);
  for (x = 0; x < mapSizeX; x++){
    for (y = 0; y < mapSizeY; y++)
      printf("%f ", map[x][y]);
    printf("\n");
  }

  
  exit(0);
}

  
  


  
