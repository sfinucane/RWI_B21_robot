
#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <values.h>

#include "general.h"
#include "map.h"
#include "inline.h"
#include "function.h"
#include "graphic.h"
#include "allocate.h"


extern FILE *logFile;
extern mapParameters globalMapParameters;
  


float
setMapDifference( probabilityGrid *map1,
		  probabilityGrid *map2,
		  int offsetX, int offsetY,
		  probabilityGrid *map3)
{
  int x, x2, y, y2;
  float error = 0;
  
  x2=-offsetX;
  for (x = 0; x < map3->sizeX; x++,x2++){
    y2=-offsetY;
    for (y = 0; y < map3->sizeY; y++, y2++){
      if (x < map1->sizeX && x2 < map2->sizeX && x2 >= 0 &&
	  y < map1->sizeY && y2 < map2->sizeY && y2 >= 0)
	if (map1->prob[x][y] >= 0 && map2->prob[x2][y2] >= 0)
	  map3->prob[x][y] = fAbs(map1->prob[x][y] - map2->prob[x2][y2]);
	else
	  map3->prob[x][y] = 0.0;
      else 
	map3->prob[x][y] = 0.0;
      error += fSqr(map3->prob[x][y]);
    }  
  }
  return error;
}
  
void
setCorners( probabilityGrid *map){
  if (map->prob[0][0] < 0)
    map->prob[0][0] = MINIMUM_PROBABILITY;
  if (map->prob[map->sizeX-1][map->sizeY-1] < 0)
    map->prob[map->sizeX-1][map->sizeY-1] = MINIMUM_PROBABILITY;
}



void
saveMaps( probabilityGrid *map1, char *fileName1,
	  probabilityGrid *map2, char *fileName2,
	  int offsetX, int offsetY){
  int startX, startY, endX, endY;
  probabilityGrid map = *map1;
  int x,y;
  
  /* map 1 */
  startX = iMax(0, offsetX);
  startY = iMax(0, offsetY);

  endX = iMin(map1->sizeX, map2->sizeX+offsetX);
  endY = iMin(map1->sizeY, map2->sizeY+offsetY);

  fprintf(stderr, "map1: startX %d, startY %d, endX %d, endY %d\n" ,
	  startX, startY, endX, endY);

  map.sizeX = endX - startX;
  map.sizeY = endY - startY;
  map.prob = (mapProbability **) allocate2D(map.sizeX, map.sizeY,MAP_PROBABILITY);
  
  for ( x = 0; x < map.sizeX; x++)
    for (y = 0; y < map.sizeY; y++){
      map.prob[x][y] = map1->prob[x+startX][y+startY];
    }

  setCorners( &map);

  fprintf(stderr, "Dumping map %s.aligned ...", fileName1);
  writeGridMap(fileName1, ".aligned", &map);
  fprintf(stderr, "done.\n");
  
  
  /* map 2 */
  startX = iMax(0, -offsetX);
  startY = iMax(0, -offsetY);
  
  endX = iMin(map1->sizeX-offsetX, map2->sizeX);
  endY = iMin(map1->sizeY-offsetY, map2->sizeY);
  
  fprintf(stderr, "map2: startX %d, startY %d, endX %d, endY %d\n" ,
	  startX, startY, endX, endY);
  
  for ( x = 0; x < map.sizeX; x++)
    for (y = 0; y < map.sizeY; y++){
      map.prob[x][y] = map2->prob[x+startX][y+startY];
    }

  setCorners( &map);
  
  fprintf(stderr, "Dumping map %s.aligned ... ", fileName2);
  writeGridMap(fileName2, ".aligned", &map);
  fprintf(stderr, "done.\n");
  
  free2D((void **) map.prob, map.sizeX, MAP_PROBABILITY);
}


int
main( int argc, char *argv[] )
{
  int argCount = 0;
  
  
  probabilityGrid map1;
  probabilityGrid map2;
  probabilityGrid map3;


  int offsetX = 0, offsetY = 0;
  gridWindow *win1, *win2, *win3;
  EZX_EventType Event;
  char c=' ';
  
  char *fileName1, *fileName2;

  /* set global variables of LOCALIZE needed here */
  logFile = NULL;
  globalMapParameters.unknown = -1.0;

  
  if (argc != 3)
    {
      fprintf(stderr,
	      "usage: %s map1 map2\n",
	      argv[0]);
      exit(0);
    }      
  
  argCount++;
  fileName1 = argv[argCount];
  
  argCount++;
  fileName2 = argv[argCount];
  
  if (!readGridMap(fileName1, "", &map1))
    exit(-1);

  if (!readGridMap(fileName2, "", &map2))
    exit(-1);

  map3.sizeX = iMax(map1.sizeX, map2.sizeX);
  map3.sizeY = iMax(map1.sizeY, map2.sizeY);
  map3.offsetX = map3.offsetY = 0;
  map3.resolution = map1.resolution;
  
  map3.prob = (mapProbability **) allocate2D(map3.sizeX, map3.sizeY,
					     MAP_PROBABILITY);

  win3 = createMapWindow(&map3, "Difference", 0, 0, 2);
  win1 = createMapWindow(&map1, fileName1, 0, map3.sizeY*2, 2);
  win2 = createMapWindow(&map2, fileName2, map1.sizeX*2, map3.sizeY*2, 2);
  
  displayMapWindow( &map1, win1);
  displayMapWindow( &map2, win2);


  if (0) {
    int x,y;
    for ( x = 0; x < map1.sizeX; x++)
      for (y = 0; y < map1.sizeY; y++){
	if (map1.prob[x][y] < 0 || map1.prob[x][y] > 1)
	  fprintf(stderr, "%d %d: %f\n", x,y,map1.prob[x][y]);
      }
  }
  
  while (c != 'q'){
    float error;
    error = setMapDifference( &map1, &map2, offsetX, offsetY, &map3);
    fprintf(stderr, "offsetX: %d   offsetY: %d, error: %f\n",
	    offsetX, offsetY, error);
    displayMapWindow( &map3, win3);

    do {
      EZX_GetEvent( &Event );
    }
    while (Event.type != EZX_Event_was_Key_Press);

    c = (char) Event.Key;
    switch (c){
    case 'b':
    case 'l':
      offsetX--;
      break;
    case 'f':
    case 'r':
      offsetX++;
      break;
    case 'n':
    case 'u':
      offsetY++;
      break;
    case 'p':
    case 'd':
      offsetY--;
      break;
    case 'c':
      offsetY=offsetX=0;
      break;
    case 's':
      saveMaps(&map1, fileName1, &map2, fileName2, offsetX, offsetY);
      break;
    default:
      break;
    }
  }

  exit(0);
} 

