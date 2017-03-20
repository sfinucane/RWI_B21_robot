#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#define OCCU 0
#define ROOM 1
#define HALL 2

#define cmPerCell 200.0
#define cellOffset 100.0

#define doorWallDepth 30.0
#define doorWallWidth 40.0

/* Example environments from IROS'96 (p. 969) */
#define envA

#ifdef envA
#define xDim 14
#define yDim 12

int tmp[yDim][xDim] = { 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			2, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0,
			2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0,
			2, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 1,
			2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0,
			2, 0, 0, 0, 0, 0, 2, 2, 1, 0, 0, 0, 2, 0,
			2, 2, 2, 2, 2, 0, 2, 0, 0, 0, 1, 0, 2, 0,
			1, 0, 0, 0, 2, 0, 2, 2, 2, 2, 2, 2, 2, 0,
			0, 0, 0, 0, 2, 2, 2, 0, 0, 0, 0, 1, 2, 0,
			0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0,
			0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0};

#endif


#ifdef envB
#define xDim 17
#define yDim 5

  int tmp[yDim][xDim] = { 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0,
			  0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0,
			  1, 2, 2, 2, 0, 0, 0, 2, 2, 2, 2, 0, 0, 0, 0, 2, 1,
			  0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0,
			  0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0};

#endif

int
main( int argc, char** argv)
{

  int x, y;
  int map[xDim][yDim];
  
  for ( x = 0; x < xDim; x++) 
    for ( y = 0; y < yDim; y++) 
      map[x][y] = tmp[y][x];


  fprintf( stderr, "MAP 0.0 0.0 %f %f\n",
	   xDim * cmPerCell + cellOffset, yDim * cmPerCell + cellOffset);
	 
  fprintf( stderr, "ROBOT 400.0 300.0 0.0\n");

  /* Set the walls. */
  for ( x = 0; x < xDim; x++) 
    for ( y = 0; y < yDim; y++) 
      if ( map[x][y] == OCCU)
	fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		 x * cmPerCell + cellOffset, y * cmPerCell + cellOffset, cmPerCell, cmPerCell, 0.0);

  /* Set the openings. */
  for ( x = 0; x < xDim; x++) 
    for ( y = 0; y < yDim; y++) 
      
      if ( map[x][y] == ROOM) {
	
	/* LEFT */
	if ( x > 0)
	  if ( map[x-1][y] == HALL) {
	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     x * cmPerCell + doorWallDepth / 2,
		     y * cmPerCell + doorWallWidth / 2,
		     doorWallDepth, doorWallWidth, 0.0);

	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     x * cmPerCell + doorWallDepth / 2,
		     (y+1) * cmPerCell - doorWallWidth / 2,
		     doorWallDepth, doorWallWidth, 0.0);
	  }
	
	/* RIGHT */
	if ( x < xDim - 1) 
	  if ( map[x+1][y] == HALL) {
	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     (x+1) * cmPerCell - doorWallDepth / 2,
		     y * cmPerCell + doorWallWidth / 2,
		     doorWallDepth, doorWallWidth, 0.0);
	    
	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     (x+1) * cmPerCell - doorWallDepth / 2,
		     (y+1) * cmPerCell - doorWallWidth / 2,
		     doorWallDepth, doorWallWidth, 0.0);
	  }
	
	/* UPPER */
	if ( y < yDim - 1) 
	  if ( map[x][y+1] == HALL) {
	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     x * cmPerCell + doorWallWidth / 2,
		     (y+1) * cmPerCell - doorWallDepth / 2,
		     doorWallWidth, doorWallDepth, 0.0);

	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     (x+1) * cmPerCell - doorWallWidth / 2,
		     (y+1) * cmPerCell - doorWallDepth / 2,
		     doorWallWidth, doorWallDepth, 0.0);
	  }

	/* LOWER */
	if ( y > 0)
	  if ( map[x][y-1] == HALL)	 {
	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     x * cmPerCell + doorWallWidth / 2,
		     y * cmPerCell + doorWallDepth / 2,
		     doorWallWidth, doorWallDepth, 0.0);

	    fprintf( stderr, "RECTANGLE %f %f %f %f %f\n",
		     (x+1) * cmPerCell - doorWallWidth / 2,
		     y * cmPerCell + doorWallDepth / 2,
		     doorWallWidth, doorWallDepth, 0.0);
	  }
      }

}
