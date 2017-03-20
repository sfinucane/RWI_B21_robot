
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/SIM2FIG.c,v $
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
 * $Log: SIM2FIG.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.21  2000/06/14 14:00:31  haehnel
 * *** empty log message ***
 *
 * Revision 1.20  2000/05/14 11:15:24  schulz
 * Added Error ellipses to -pos Option
 *
 * Revision 1.19  2000/03/18 17:16:47  wolfram
 * Fixed a bug!
 *
 * Revision 1.18  2000/03/18 15:26:46  wolfram
 * Added option for line style
 *
 * Revision 1.17  2000/02/09 15:38:51  wolfram
 * Added option to get colored trajectories
 *
 * Revision 1.16  1999/12/01 08:55:53  schulz
 * Added a -arrowfile option to SIM2FIG
 *
 * Revision 1.15  1999/03/22 21:49:33  wolfram
 * Files now compile again.
 *
 * Revision 1.14  1998/11/09 21:34:17  wolfram
 * Added sensings to trajectory including a -nomap option
 *
 * Revision 1.13  1998/10/26 08:53:04  wolfram
 * Fixed bugs, added things
 *
 * Revision 1.12  1998/10/19 19:41:25  fox
 * *** empty log message ***
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


#define MAX_NUMBER_OF_OBJECTS 10000
#define MAP_MARK "MAP"
#define RECTANGLE_MARK "RECTANGLE"
#define DOOR_MARK "DOOR"
#define ROBOT_MARK "ROBOT"
#define CUBE_MARK "CUBE"
#define CYLINDER_MARK "CYLINDER"
#define SKIP 1
#define MAXDEPTH 500


#define BUFFLEN 1024

#define TRUE 1
#define FALSE 0
#define CUBE 0
#define CYLINDER 1

#define MAX_INTENSITY 20

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

int useColor = FALSE;
int depth= 0;

static float inside_x[4];
static float inside_y[4];

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

void
rotpnt(float angle, float *x, float *y) 
{ 
    float t = *x; 
    *x = *x * cos(angle) - *y * sin(angle); 
    *y = t  * sin(angle) + *y * cos(angle); 
}


void
absolute_points(rectangle *rect)
{
  int i;
  inside_x[0] = rect->width/2;
  inside_y[0] = -rect->depth/2;
  rotpnt(rect->rot,&inside_x[0],&inside_y[0]);
  inside_x[1] = rect->width/2;
  inside_y[1] = rect->depth/2;
  rotpnt(rect->rot,&inside_x[1],&inside_y[1]);
  inside_x[2] = -rect->width/2;
  inside_y[2] = rect->depth/2;
  rotpnt(rect->rot,&inside_x[2],&inside_y[2]);
  inside_x[3] = -rect->width/2;
  inside_y[3] = -rect->depth/2;
  rotpnt(rect->rot,&inside_x[3],&inside_y[3]);
  for(i = 0; i <4; i++) {
    inside_x[i] += rect->centerX;
    inside_y[i] += rect->centerY;
  }
}

int
inhalfplane(float xp, float yp, float x1, float y1, float x2, float y2)
{
    float m,c;
  if(x1 == x2) {
    if(y1 <= y2)
      return (xp <= x1);
    else
      return (xp >= x1);
  }
  else {
    if( x1 < x2 ) {
      m  = (y2 - y1) / (x2 - x1);
      c = y1 - m*x1;
      return (yp >= m*xp+c);
    }
    else {
      m  = (y1 - y2) / (x1 - x2);
      c = y1 - m*x1;
      return (yp <= m*xp+c);
    }
  }
}

int
inside(float xi, float yi, rectangle* rect)
{
    int i;
    for(i = 0; i < 4; i++) {
	if(!inhalfplane(xi,yi,
			inside_x[i],inside_y[i],
			inside_x[(i+1)%4],inside_y[(i+1)%4]))
	    return FALSE;
    }
    return TRUE;
}

int
round(float x)
{
  return (int) (x + 0.5);
}



int
iMin(int x, int y)
{
  if (x < y)
    return(x);
  else
    return(y);
}

int
iMax(int x, int y)
{
  if (x > y)
    return(x);
  else
    return(y);
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
xCoord(float x, float mapSizeX, float scale){
  return round(x * scale);
}


int
yCoord(float y, float mapSizeY, float scale){
  return round((mapSizeY - y) * scale);
}


void
writeRectangle(rectangle rect, float mapSizeX, float mapSizeY,
	       float scaleFactor, int intensity)
{
  int i;
  int color = 0;
    
  absolute_points( &rect);

  if ( useColor)
    color = 1;

  if ( fabs(rect.rot) <= 0.001)
    printf("2 2 0 0 %d %d %d 0 %d 0.000 0 0 -1 0 0 5\n",
	   color, color, depth, intensity); 
  else
    printf("2 1 0 0 %d %d %d 0 %d 0.000 0 0 -1 0 0 5\n",
	   color, color, depth, intensity);
  
  for (i = 0; i < 5; i++)
    printf("%d %d ",
	   xCoord(inside_x[i % 4], mapSizeX, scaleFactor),
	   yCoord(inside_y[i % 4], mapSizeY, scaleFactor));

  printf("\n");
}

void
writeTransProbMark(float mapSizeX, float mapSizeY, float scaleFactor,
		   float x, float y,
		   float r0, float r1, float r2, float r3,
		   float r4, float r5, float r6, float r7)
{
  int xc = xCoord(x, mapSizeX, scaleFactor);
  int yc = yCoord(y, mapSizeY, scaleFactor);
  int xp, yp;
  float rot = 0;
  float inc_rot = M_PI/4;
  float l = 80;
  
  xp = xCoord(x+r0*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r0*l*sin(rot), mapSizeY, scaleFactor);
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r1*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r1*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r2*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r2*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r3*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r3*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r4*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r4*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r5*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r5*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r6*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r6*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;

  xp = xCoord(x+r7*l*cos(rot), mapSizeX, scaleFactor);
  yp = yCoord(y+r7*l*sin(rot), mapSizeY, scaleFactor);  
  printf("2 1 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 2\n");
  printf("%d %d %d %d\n",
	 xc, yc, xp, yp);
  rot += inc_rot;
  
}


void
writeCircle(rectangle rect, float mapSizeX, float mapSizeY,
	    float scaleFactor, int intensity)
{
  int x = xCoord(rect.centerX, mapSizeX, scaleFactor),
     y = yCoord(rect.centerY, mapSizeY, scaleFactor),
    radius =  xCoord(rect.width, mapSizeY, scaleFactor);
  
  int color = 0;
  
  if (useColor)
    color = 1;
  
  printf("1 3 0 0 %d %d %d 0 %d 0.000 1 0.000 ",
	 color, color, depth, intensity); 

  printf("%d %d %d %d %d %d %d %d\n",
	 x, y, radius, radius, x, y, x+radius, y+radius);

}




float
fSqr( float x)
{
  return x*x;
}

 

float
distance(float x1, float y1, float x2, float y2)
{
  return sqrt(fSqr(x1 - x2) + fSqr(y1 - y2));
}





#define MAX_POSITION_NUMBER 100000
#define MAX_POSITION_NUMBER 100000

int
main( int argc, char *argv[] )
{
  FILE *ifp;
   
  char line[BUFFLEN];
  int markLength;
  int found, foundP;
  int noMap = FALSE;
  float simRobotX, simRobotY, simRobotO;
  float mapX1, mapX2, mapY1, mapY2;
  float mapSizeX, mapSizeY;
  float desiredHeight = 0.0;
  int heightSpecified = FALSE;
  char *positionLog = NULL, *markerFileName = NULL, *plotFileName = NULL;
  char *arrowFileName = NULL;
  char *sampleFile = NULL;
  char *transProbFileName = NULL;
  rectangle rect,object[MAX_NUMBER_OF_OBJECTS];
  int numberOfObjects = 0;
  int i,j, intensity = 20;
  int haveToRewind = FALSE;
  int plotBeams = FALSE;
  int minX, minY, maxX, maxY;
  int numbered = FALSE;
  int trajectoryColor = -1;
  int trajectoryStyle = 0; /* solid */
  
  float scaleFactor = 4.5;
   
  if ((argc < 2) || (argc > 19))
    {
      fprintf(stderr, "usage: %s infile [-nomap] [-scale s] [-pos positionlog] [-sam samples] [-height h] [-colored] [-intensity (0-20)] [-depth d] [-marker markerfile] [-plotfile plotfile] [-beams] [-numbered] [-pathcolor n] [-pathstyle n]\n", argv[0]);
      exit(0);
    }      
   
   
  if ((ifp = fopen(argv[1],"r")) == NULL) {
    fprintf(stderr,"ERROR: Could not open simulator file '%s'!\n",argv[1]);
    exit(0);
  }

  for (i=2; i<argc; i++) {
    if ((strcmp(argv[i],"-pos")==0)) {
      if ( i < argc - 1)
	positionLog = argv[++i];
      else {
	fprintf( stderr, "ERROR: logFile must follow keyword -pos.\n");
	exit;
      }
    }
    else if ((strcmp(argv[i],"-sam")==0)) {
      if ( i < argc - 1) {
	sampleFile = argv[++i];
	fprintf( stderr, "\n---- Sample file: %s ----\n\n", sampleFile);
      }
      else {
	fprintf( stderr, "ERROR: sampleFile must follow keyword -sam.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-height")==0) {
      if ( i < argc - 1) {
	desiredHeight = atof(argv[++i]);
	heightSpecified = TRUE;
	fprintf( stderr, "Height: %f\n", desiredHeight);
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword -height.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-scale")==0) {
      if ( i < argc - 1) {
	scaleFactor = atof(argv[++i]);
	fprintf( stderr, "Scale: %f\n", scaleFactor);
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword -scale.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-depth")==0) {
      if ( i < argc - 1) {
	depth = atoi(argv[++i]);
	fprintf( stderr, "Depth: %d\n", depth);
      }
      else {
	fprintf( stderr, "ERROR: integer must follow keyword -depth.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-intensity")==0) {
      if ( i < argc - 1) {
	intensity = atoi(argv[++i]);
	if ( intensity > MAX_INTENSITY) {
	  fprintf( stderr, "ERROR: intensity %d must be in [0:20].\n", intensity);
	  exit;
	}
	fprintf( stderr, "Intensity: %d\n", intensity);
      }
      else {
	fprintf( stderr, "ERROR: integer must follow keyword -intensity.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-pathcolor")==0) {
      if ( i < argc - 1) {
	trajectoryColor = atoi(argv[++i]);
	fprintf( stderr, "pathColor: %d\n", trajectoryColor);
      }
      else {
	fprintf( stderr, "ERROR: integer must follow keyword -pathcolor.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-pathstyle")==0) {
      if ( i < argc - 1) {
	trajectoryStyle = atoi(argv[++i]);
	fprintf( stderr, "pathColor: %d\n", trajectoryStyle);
      }
      else {
	fprintf( stderr, "ERROR: integer must follow keyword -pathstyle.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-colored")==0) {
      fprintf( stderr, "Use color.\n");
      useColor = TRUE;
    }
    else if (strcmp(argv[i],"-marker")==0) {
      if ( i < argc - 1)
	markerFileName = argv[++i];
      else {
	fprintf( stderr, "ERROR: markerFile must follow keyword -marker.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-plotfile")==0) {
      if ( i < argc - 1)
	plotFileName = argv[++i];
      else {
	fprintf( stderr, "ERROR: plotfile must follow keyword -plotfile.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-arrowfile")==0) {
      if ( i < argc - 1)
	arrowFileName = argv[++i];
      else {
	fprintf( stderr, "ERROR: arrowfile must follow keyword -arrowfile.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-beams")==0) {
      fprintf( stderr, "Plotting beams\n");
      plotBeams = TRUE;
    }
    else if (strcmp(argv[i],"-nomap")==0) {
      fprintf( stderr, "Not plotting map\n");
      noMap = TRUE;
    }
    else if (strcmp(argv[i],"-numbered")==0) {
      fprintf( stderr, "Plotting numbered\n");
      numbered = TRUE;
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[i]);
      getchar();
    }
  }
    
  numberOfObjects = markLength = 0;
   
   /* scanning for map size and robot position */
  found = FALSE;
  foundP = FALSE;

  while (!(found && foundP) && (fgets(line,BUFFLEN,ifp) != NULL)){
    if (strncmp(line,MAP_MARK,strlen(MAP_MARK)) == 0){
      markLength = strlen(MAP_MARK);
      if (sscanf(&line[markLength],"%f %f %f %f", &mapX1, &mapY1,
		 &mapX2, &mapY2) == 4){
	fprintf(stderr, "Map size: (%fx%f) -> (%fx%f)\n", mapX1, mapY1,
		mapX2, mapY2);
	found = TRUE;
      }
    }
    else if (strncmp(line,ROBOT_MARK,strlen(ROBOT_MARK)) == 0){
      markLength = strlen(ROBOT_MARK);
      if (sscanf(&line[markLength],"%f %f %f", &simRobotX, &simRobotY,
		 &simRobotO) == 3){
	fprintf(stderr, "Simulator Robot Position: X=%f Y=%f O=%f\n", 
		simRobotX, simRobotY, simRobotO);
	foundP = TRUE;
      }
    }
    else if ( strlen( line) > 1)
      haveToRewind = TRUE;
  }

  if (!found){
    fprintf(stderr, "Error: Map size not found!\n");
    exit(1);
  }

  if (!foundP){
    fprintf(stderr, "Warning: Robot Position not found!\n");
    exit(1);
  }

  fprintf(stderr, "Map size: (%fx%f) -> (%fx%f)\n", mapX1, mapX2,
          mapY1, mapY2);
   
  mapSizeX =  (mapX2 + 1 - mapX1);
  mapSizeY = (mapY2 + 1 - mapY1);
 
  printf("#FIG 3.2\n");
  printf("Portrait\n");
  printf("Center\n");
  printf("Metric\n");
  printf("A4\n");
  printf("100.0\n");
  printf("Single\n");
  printf("-2\n");        
  printf("1200 2\n");
  
  if ( haveToRewind)
    rewind( ifp);
  
  while ((fgets(line,BUFFLEN,ifp) != NULL)){
    found = FALSE;
    if (strncmp(line,RECTANGLE_MARK,
		markLength = strlen(RECTANGLE_MARK)) == 0){
      if (sscanf(&line[markLength],"%f %f %f %f %f", &rect.centerX,
		 &rect.centerY,
		 &rect.width,
		 &rect.depth,
		 &rect.rot) == 5){
	found = TRUE;
      }
      else if (sscanf(&line[markLength],"%f %f %f %f", &rect.centerX,
		      &rect.centerY,
		      &rect.width,
		      &rect.depth) == 4){
	rect.rot = 0.0;
	found = TRUE;
	rect.type = CUBE;
      }
    }
    else if (strncmp(line,CUBE_MARK,
		     markLength = strlen(CUBE_MARK)) == 0){
      if (sscanf(&line[markLength],"%f %f %f %f %f %f %f", &rect.centerX,
		 &rect.centerY,
		 &rect.centerZ,
		 &rect.width,
		 &rect.depth,
		 &rect.height,
		 &rect.rot) == 7){
	if ( ! heightSpecified
	     || 
	     ((desiredHeight <= rect.centerZ + rect.height)
	     &&
	     (desiredHeight >= rect.centerZ - rect.height))) {
	  found = TRUE;
	  rect.type = CUBE;
	}
      }
    }
    else if (strncmp(line,CYLINDER_MARK,
		     markLength = strlen(CYLINDER_MARK)) == 0){
      if (sscanf(&line[markLength],"%f %f %f %f %f",
		 &rect.centerX,
		 &rect.centerY,
		 &rect.centerZ,
		 &rect.width,
		 &rect.height) == 5){
	if ( ! heightSpecified
	     || 
	     ((desiredHeight <= rect.centerZ + rect.height)
	      &&
	      (desiredHeight >= rect.centerZ - rect.height))) {
	  found = TRUE;
	  rect.depth = rect.width;
	  rect.rot = 0.0;
	  rect.type = CYLINDER;
	}
      }
    }
    else if (strncmp(line, DOOR_MARK,
		     markLength = strlen(DOOR_MARK)) == 0) {
      if (sscanf(&line[markLength],"%f %f %f %f %f %f %f", &rect.centerX,
		 &rect.centerY,
		 &rect.centerZ,
		 &rect.width,
		 &rect.depth,
		 &rect.height,
		 &rect.rot) == 7){
	rect.centerX += (0.5*rect.width)*cos(rect.rot) 
	  - (0.5*rect.depth)*sin(rect.rot);
	rect.centerY += (0.5*rect.width)*sin(rect.rot) 
	  + (0.5*rect.depth)*cos(rect.rot);
	if ( ! heightSpecified
	     || 
	     ((desiredHeight <= rect.centerZ + rect.height)
	      &&
	      (desiredHeight >= rect.centerZ - rect.height))) {
	  found = TRUE;
	  rect.type = CUBE;
	}
      }
    }
    else if ( strlen( line) > 1)
      fprintf( stderr, "UNKNOWN: %s\n", line);
    
    
    if (found){
      if (numberOfObjects > MAX_NUMBER_OF_OBJECTS){
	fprintf(stderr, "Error: too many objects in file, fix that first\n");
	exit(0);
      }
      rect.centerX -= mapX1;
      rect.centerY -= mapY1;
      object[numberOfObjects++] = rect;
    }
  }
  

  
  fprintf(stderr, "Found %d objects\n", numberOfObjects);
  fclose(ifp);
  
  minX = minY = 32767;
  maxX = maxY = 0;
  
  for (i = 0; i < numberOfObjects; i++)    {
    int x, y;
    absolute_points( &(object[i]));
    for (j = 0; j < 5; j++){
      x = xCoord(inside_x[i % 4], mapSizeX, scaleFactor);
      y = yCoord(inside_y[i % 4], mapSizeY, scaleFactor);
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
  }
  
  if (!noMap){
    printf("6 %d %d %d %d\n", minX, minY, maxX, maxY);
    for (i = 0; i < numberOfObjects; i++)    {
      if (object[i].type == CUBE)
	writeRectangle(object[i], mapSizeX, mapSizeY, scaleFactor, intensity);
      else if (object[i].type == CYLINDER)
      writeCircle(object[i], mapSizeX, mapSizeY, scaleFactor, intensity);
    }
    printf("-6\n");
  }
  
  if (positionLog != NULL){
    int count, numberOfPositions, skipCount = 0;
    float posX[MAX_POSITION_NUMBER],posY[MAX_POSITION_NUMBER];
    float radiusA[MAX_POSITION_NUMBER],radiusB[MAX_POSITION_NUMBER],
      rotation[MAX_POSITION_NUMBER];
    float time;
    if ((ifp = fopen( positionLog,"r")) == NULL) {
      fprintf(stderr,"ERROR: Could not open position file '%s'!\n",
	      positionLog);
      exit(0);
    }
    
    
    numberOfPositions = 0;
    while (!feof(ifp) && numberOfPositions < MAX_POSITION_NUMBER){
      fgets(line,BUFFLEN,ifp);
      if (sscanf(line, "%f %f %f %f %f", 
		 &posX[numberOfPositions],
		 &posY[numberOfPositions],
		 &radiusA[numberOfPositions],
		 &radiusB[numberOfPositions],
		 &rotation[numberOfPositions]) == 5){
	if (skipCount % SKIP == 0)
	  numberOfPositions++;
	skipCount++;
      }
      else if (sscanf(line, "%f %f %f", &time, &posX[numberOfPositions],
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
    
    if (numberOfPositions > 0){
      float gapLength = 0.000;
      /* Dump the path as a polyline. */
      
      if (trajectoryStyle != 0)
	gapLength = 4.000;
      
      printf("2 1 %d 1 %d  7 0 0 -1 %.3f 0 0 -1 0 0 %d\n      ",
	     trajectoryStyle,
	     trajectoryColor,
	     gapLength,
	     numberOfPositions);
      
      for (count = 0; count < numberOfPositions; count++) 	
	printf("%d %d ", round(posX[count] * scaleFactor),
	       round((mapSizeY - posY[count]) * scaleFactor));
      
      printf("\n");
    }
    
	/* Indicate the start and the end of the path. */
	{
	  int intPosX = round(posX[0] * scaleFactor);
	  int intPosY = round((mapSizeY - posY[0]) * scaleFactor);
	  int radius = 7 * scaleFactor;
	  if (radius < 20) radius = 20;
	  
	  //	  printf("1 3 0 1 -1 7 0 0 -1 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
	  printf("1 3 0 1 -1 0 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		 intPosX, intPosY, radius, radius, intPosX, intPosY,
		 intPosX + radius, intPosY + radius);
	  
	  intPosX = round(posX[numberOfPositions - 1 ] * scaleFactor);
	  intPosY = round((mapSizeY - posY[numberOfPositions -1 ])
			  * scaleFactor);
	  radius = radius / 4;
	  if (radius < 20) radius = 20;
	  
	  printf("1 3 0 1 -1 0 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		 intPosX, intPosY, radius, radius, intPosX, intPosY,
		 intPosX + radius, intPosY + radius);
	  
	}

    for (count = 0; count < numberOfPositions; count++) {
      int intPosX = round(posX[count] * scaleFactor);
      int intPosY = round((mapSizeY - posY[count]) * scaleFactor);
      int intRA = round(radiusA[count]*scaleFactor);
      int intRB = round(radiusB[count]*scaleFactor);
      printf("1 2 0 1 0 7 100 0 -1 0.000 1 %f %d %d %d %d %d %d %d %d\n", 
	     rotation[count], intPosX, intPosY, intRA, intRB, 
	     intPosX-intRA, intPosY, intPosX+intRB, intPosY);
    }
    
    fclose(ifp);
  }
  
  if (arrowFileName != NULL){
    if ((ifp = fopen( arrowFileName,"r")) == NULL) {
      fprintf(stderr,"ERROR: Could not open arrowfile '%s'!\n", plotFileName);
    }
    else {
      while (!feof(ifp)){
	char position[BUFFLEN];
	float posX, posY, posRot, speed;
	int intPosX, intPosY, radius = round(26.0*scaleFactor);
	fgets(line,BUFFLEN,ifp);
	if (line != NULL){
	  if (sscanf( line, "%f %f %f %f", &posX, &posY, &posRot, &speed) == 4){
	    intPosX = round(posX * scaleFactor);
	    intPosY = round((mapSizeY - posY) * scaleFactor);
	    speed *= scaleFactor;
	    printf("2 1 0 1 0 7 %d 0 -1 0.000 0 0 -1 1 0 2 \n      2 1 1.00 60.00 120.00\n         %d %d %d %d\n",
		   iMax(0,MAXDEPTH-1),
		   intPosX, intPosY, (int) (intPosX + round(speed * cos(posRot))),
		   (int) (intPosY - round(speed * sin(posRot))));
      
	  }
	}
      }
      fclose(ifp);
    }
  }
  if (plotFileName != NULL){
    if ((ifp = fopen( plotFileName,"r")) == NULL) {
      fprintf(stderr,"ERROR: Could not plotfile '%s'!\n", plotFileName);
    }
    else {
      int count = 1, startDepth = MAXDEPTH-1, fontSize = 18;
      while (!feof(ifp)){
	char position[BUFFLEN];
	float posX, posY, posRot;
	int intPosX, intPosY, radius = round(26.0*scaleFactor);
	fgets(line,BUFFLEN,ifp);
	if (line != NULL){
	  if (sscanf( line, "%f %f %f", &posX, &posY, &posRot) == 3){
	    intPosX = round(posX * scaleFactor);
	    intPosY = round((mapSizeY - posY) * scaleFactor);
	    printf("1 3 0 1 0 7 %d 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n", iMax(0, startDepth--),
		   intPosX, intPosY, radius, radius, intPosX, intPosY,
		   intPosX + radius, intPosY + radius);
	    printf("2 1 0 1 0 7 %d 0 -1 0.000 0 0 -1 0 0 2 \n      %d %d %d %d\n",
		   iMax(0,startDepth--),
		   intPosX, intPosY, (int) (intPosX + round(radius * cos(posRot))),
		   (int) (intPosY - round(radius * sin(posRot))));
	    if (numbered)
	      printf("4 0 -1 0 0 0 %d 0.0000 4 255 255 %d %d %d\\001\n",
		   fontSize,
		   intPosX + round((3.0*radius) * sin(posRot)),
		   intPosY + round((3*fontSize + 3.0*radius) * cos(posRot)),
		   count++);
	    while((fgets(line, BUFFLEN, ifp) != NULL) && strlen(line) > 2){
	      float beamStartX, beamStartY, beamEndX, beamEndY;
	      if (sscanf( line, "%f %f %f %f", &beamStartX,
			  &beamStartY, &beamEndX, &beamEndY) == 4){
		printf("2 1 0 1 0 7 %d 0 -1 0.000 0 0 -1 0 0 2 \n      ",
		       MAXDEPTH);
		printf("%d %d %d %d\n",
		       round( beamStartX * scaleFactor),
		       round( (mapSizeY - beamStartY) * scaleFactor),
		       round( beamEndX * scaleFactor),
		       round( (mapSizeY - beamEndY) * scaleFactor));
	      }
	    }
	  }
	}
	
      }
      fprintf(stderr, "Robots: %d\n", count-1);
      fclose(ifp);
    }
  }
  
  if (sampleFile != NULL){
    int count, numberOfPositions, skipCount = 0;
    float posX, posY, prob;
    float time;
    int minX = 100000, maxX = -10000;
    int minY = 100000, maxY = -10000;
    float probSum = 0.0, maxProb = 0.0;
    
    /* Scan for min and max values. */
    if ((ifp = fopen( sampleFile, "r")) == NULL) {
      fprintf(stderr,"ERROR: Could not open sample file '%s'!\n",
	      sampleFile);
      exit(0);
    }
    while (!feof(ifp)){
      fgets(line,BUFFLEN,ifp);
      if (sscanf(line, "%f %f %f", &prob, &posX, &posY) == 3){
	if ( posX < minX)
	  minX = posX;
	if ( posX > maxX)
	  maxX = posX;
	if ( posY < minY)
	  minY = posY;
	if ( posY > maxY)
	  maxY = posY;
	if ( prob > maxProb)
	  maxProb = prob;
	probSum += prob;
      }
    }

    fprintf( stderr, "Values: x [%d %d] y [%d %d] prob %f.\n",
	     minX, maxX, minY, maxY, maxProb);
    
    printf("6 %d %d %d %d\n", minX, minY, maxX, maxY);

    /* Now we got the ranges. Rescan the file. */
    if (! fseek( ifp, 0, SEEK_SET)) {

      while (!feof(ifp)){
	fgets(line,BUFFLEN,ifp);
	
	if (sscanf(line, "%f %f %f", &prob, &posX, &posY) == 3){
	  int intPosX = round(posX * scaleFactor);
	  int intPosY = round((mapSizeY - posY) * scaleFactor);
	  /* int radius = scaleFactor * fNorm( prob, 0.0, maxProb, 1, 10); */
	  int radius = scaleFactor;
	  
	  printf("1 3 0 1 -1 0 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		 intPosX, intPosY, radius, radius, intPosX, intPosY,
		 intPosX + radius, intPosY + radius);
	}
      }
      printf("-6\n");
    }
    fclose(ifp);
  }
  
  
  
  if (markerFileName != NULL){
    if ((ifp = fopen( markerFileName,"r")) == NULL) {
      fprintf(stderr,"ERROR: Could not marker file '%s'!\n", markerFileName);
    }
    else {
      while (fgets(line,BUFFLEN,ifp) != NULL){
	char marker[BUFFLEN];
	float posX, posY;
	int intPosX, intPosY, radius;
	if (sscanf( line, "%s %f %f", marker, &posX, &posY) == 3){
	  intPosX = round(posX * scaleFactor);
	  intPosY = round((mapSizeY - posY) * scaleFactor);
	  radius = 20;
	  printf("1 3 0 1 -1 0 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		 intPosX, intPosY, radius, radius, intPosX, intPosY,
		 intPosX + radius, intPosY + radius);
	  printf("4 0 -1 0 0 0 24 0.0000 4 255 255 %d %d %s\\001\n",
		 intPosX + 5*radius,
		 intPosY + 5*radius,
		 marker);
		 

	}
      }
      
      fclose(ifp);
    }
  }
  if(transProbFileName != NULL) {
    float posX, posY;
    float rot0, rot1, rot2, rot3, rot4, rot5, rot6, rot7;
    if ((ifp = fopen( transProbFileName,"r")) == NULL) {
      fprintf(stderr,
	      "ERROR: Could not open transition probability table '%s'!\n",
	      transProbFileName);
    }
    else {
      while (fgets(line,BUFFLEN,ifp) != NULL) {
	if(sscanf(line,"%f %f %f %f %f %f %f %f %f %f",
		  &posX, &posY,
		  &rot0, &rot1, &rot2, &rot3,
		  &rot4, &rot5, &rot6, &rot7) == 10) {
	  writeTransProbMark(mapSizeX, mapSizeY, scaleFactor,
			     posX, posY,
			     rot0, rot1, rot2, rot3,
			     rot4, rot5, rot6, rot7);
	}
      }
    }
  }
  
  exit(0);
}






