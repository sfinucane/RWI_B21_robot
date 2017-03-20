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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/SIM2OBSTSIM.c,v $
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
 * $Log: SIM2OBSTSIM.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/06/03 11:49:12  fox
 * Museum version.
 *
 * Revision 1.2  1997/04/24 21:24:53  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.1  1997/04/22 17:58:10  wolfram
 * Added SIM2OBSTSIM.c, fixed a bug in the procedure reading the simulator map
 *
 * Revision 1.3  1997/04/15 12:02:01  wolfram
 * Nicer blue
 *
 * Revision 1.2  1997/04/15 11:41:56  wolfram
 * Changed transparent background to blue
 *
 * Revision 1.1  1997/04/14 09:13:12  haehnel
 * *** empty log message ***
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <values.h>

#include "gd.h"
#include "tcx.h"
#include "rwibase_interface.h"
#include "../localize/general.h"
#include "../ezx/EZX11.h"

#define ROBRADIUS 26.5
#define MAX_NUMBER_OF_OBJECTS 2000
#define CUBE 0
#define CYLINDER 1
#define SIMULATOR_MAP_MARK "MAP"
#define CUBE_MARK "CUBE"
#define RECTANGLE_MARK "RECTANGLE"
#define CYLINDER_MARK "CYLINDER"
#define DOOR_MARK "DOOR"
#define SIMULATOR_ROBOT_MARK "ROBOT"
#define MINIMUM_MAP_SIZE 300
#define MAP_SIZE_STEP 50
#define NUMBER_OF_CORNERS 4
#define BUFFLEN 80
#define EXPONATE_MARK "EXPONAT"
#define EXPONATE_RADIUS 9
#define MAX_NUMBER_OF_EXPONATES 100
#define FONTWIDTH 6
#define FONTHEIGHT 9

typedef struct
{
  int x;
  int y;
} windowPosition;



typedef struct {
  int initialized;
  int sizeX;
  int sizeY;
  int origsizeX;
  int origsizeY;
  int resolution;
  float offsetX;
  float offsetY;  
  float maxRealX;
  float maxRealY;
  int** prob;
} picture;


typedef struct{
  float centerX;
  float centerY;
  float centerZ;
  float width;
  float depth;
  float height;
  float rot;
} rectangle;



typedef struct{
  int number;
  realPosition pos;
} exponateType;


typedef struct{
  int numberOfExponates;
  exponateType exponate[MAX_NUMBER_OF_EXPONATES];
} exponateList;



int
round(float x)
{
  return (int) (x + 0.5);
}

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


static bool
intersection(float from1, float to1, float from2, float to2){
  return from2 <= to1 && from1 <= to2;
}



bool
readSimulatorMap( FILE *fp, 
		  char *mapname,
		  float additionalObjectSize,
		  float fromHeight,
		  float toHeight){
   char line[100];
   int markLength, cnt;
   int found, sizeFound, robotFound;
   register int x,y;
   float mapX1, mapX2, mapY1, mapY2;
   int delta;
   picture tmpMap;
   rectangle rect;
   realPosition pos;
   int type;
   exponateList expoList;

   mapX1 = mapX2 = mapY1 = mapY2 = 0.0;
   
   expoList.numberOfExponates = 0;
   sizeFound = robotFound = FALSE;
   while ((fgets(line,BUFFLEN,fp) != NULL) && !feof(fp)
	  && (!sizeFound || !robotFound)){
     fprintf(stderr, "%s", line);
     if (strncmp(line,SIMULATOR_MAP_MARK,strlen(SIMULATOR_MAP_MARK)) == 0){
       markLength = strlen(SIMULATOR_MAP_MARK);
       if (sscanf(&line[markLength],"%f %f %f %f", &mapX1, &mapY1,
		  &mapX2, &mapY2) == 4){
	 fprintf(stderr, "# Map size: (%fx%f) -> (%fx%f)\n", mapX1, mapY1,
		 mapX2, mapY2);
	 printf("%s", line);
	 sizeFound = TRUE;
       }
     }
     if (strncmp(line,SIMULATOR_ROBOT_MARK,
		 strlen(SIMULATOR_ROBOT_MARK)) == 0){
       markLength = strlen(SIMULATOR_ROBOT_MARK);
       if (sscanf(&line[markLength],"%f %f %f", &pos.x, &pos.y,
		  &pos.rot) == 3){
	 printf("%s", line);
	 robotFound = TRUE;
       }
       
     }
   }
       
   if (!sizeFound){
      fprintf(stderr, "Error: Map size not found!\n");
      return(FALSE);
   }

   if (!robotFound){
     fprintf(stderr, "Error: No robot position found!\n");
     return(FALSE);
   }
   
   rewind(fp);
   
   /* scanning for objects */
   
   cnt = 0;
   found = FALSE;
   type = CUBE;
   
   fprintf(stderr, "# Processing objects ...");

   while ((fgets(line,BUFFLEN,fp) != NULL)){
     if (strncmp(line,CUBE_MARK,
		      markLength = strlen(CUBE_MARK)) == 0){
       type = CUBE;
       if (sscanf(&line[markLength],"%f %f %f %f %f %f %f",
		  &rect.centerX,
		  &rect.centerY,
		  &rect.centerZ,
		  &rect.width,
		  &rect.depth,
		  &rect.height,
		  &rect.rot) == 7){
	 found = TRUE;
       }
       else if (sscanf(&line[markLength],"%f %f %f %f %f %f",
		       &rect.centerX,
		       &rect.centerY,
		       &rect.centerZ,
		       &rect.width,
		       &rect.depth,
		       &rect.height) == 6){
	 rect.rot = 0.0;
	 found = TRUE;
       }
     }
     else if (strncmp(line,CYLINDER_MARK,
		      markLength = strlen(CYLINDER_MARK)) == 0){
       type = CYLINDER;
       if (sscanf(&line[markLength],"%f %f %f %f %f",
		  &rect.centerX,
		  &rect.centerY,
		  &rect.centerZ,
		  &rect.width,
		  &rect.height) == 5){
	 rect.depth = rect.width;
	 rect.rot = 0.0;
	 found = TRUE;

       }
     }
     else if (strncmp(line, EXPONATE_MARK,
		      markLength = strlen(EXPONATE_MARK)) == 0){
       int n = expoList.numberOfExponates;
       if ( n < MAX_NUMBER_OF_EXPONATES
	    && sscanf(&line[markLength],"%d %f %f",
		      &expoList.exponate[n].number,
		      &expoList.exponate[n].pos.x,
		      &expoList.exponate[n].pos.y) == 3){
	 expoList.exponate[n].pos.rot = 0;
	 expoList.numberOfExponates++;
	 expoList.exponate[n].number = expoList.numberOfExponates;
       }
     }
     else if (strncmp(line,RECTANGLE_MARK,
		 markLength = strlen(RECTANGLE_MARK)) == 0){
       type = CUBE;
       if (sscanf(&line[markLength],"%f %f %f %f %f",
		  &rect.centerX,
		  &rect.centerY,
		  &rect.width,
		  &rect.depth,
		  &rect.rot) == 5){
	  rect.centerZ = 0.0;
	  rect.height = 300.0;
	  found = TRUE;
       }
       else if (sscanf(&line[markLength],"%f %f %f %f",
		       &rect.centerX,
		       &rect.centerY,
		       &rect.width,
		       &rect.depth) == 4){
	 rect.centerZ = rect.rot = 0.0;
	 rect.height = 300;
	 found = TRUE;
       }
     }

#define CENTER_Z 150.0
#define HEIGHT   300.0
     
     if (found && intersection(fromHeight, toHeight,
			       rect.centerZ - rect.height * 0.5,
			       rect.centerZ + rect.height * 0.5)){
       cnt++;
       switch (type){
       case CYLINDER:
	 rect.width += additionalObjectSize;
	 rect.depth += additionalObjectSize;
	 printf("CYLINDER %.2f %.2f %.2f %.2f %.2f\n",
		rect.centerX,
		rect.centerY,
		CENTER_Z,
		rect.width,
		HEIGHT);
	 break;
       case CUBE:
	 rect.width += 2.0 * additionalObjectSize;
	 rect.depth += 2.0 * additionalObjectSize;
	 printf("CUBE %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		rect.centerX,
		rect.centerY,
		CENTER_Z,
		rect.width,
		rect.depth,
		HEIGHT,
		rect.rot);
	 break;
       }
       found = FALSE;
     }
   }

   return(TRUE);
}


int
main( int argc, char *argv[] )
{
  FILE *ifp;
   
  int argCount;
  int resolution;
  float fromHeight, toHeight;
  float additionalObjectBorder;
  picture map;
  realPosition robotPos;
  bool displayRobot;

  char* inFileName;
  char* outFileName;

  
  argCount = 0;
  if (argc != 5)
    {
      fprintf(stderr,
	      "usage: %s addSize fromHeight toHeight inFile\n",
	      argv[0]);
      exit(0);
    }      


  argCount++;
  additionalObjectBorder = atof(argv[argCount]);
  if (additionalObjectBorder < 0)
    {
      fprintf(stderr, "Wrong border: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "Additional Object Border: %fcm\n",
	    additionalObjectBorder);


  argCount++;
  fromHeight = atof(argv[argCount]);
  if (fromHeight < 0)
    {
      fprintf(stderr, "Wrong fromHeight: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "fromHeight: %f\n",
	    fromHeight);

  argCount++;
  toHeight = atof(argv[argCount]);
  if (toHeight < 0)
    {
      fprintf(stderr, "Wrong toHeight: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "toHeight: %f\n",
	    toHeight);


  argCount++;
  inFileName = argv[argCount];
  if ((ifp = fopen(inFileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open simulator file '%s'!\n",
	    inFileName);
    exit(0);
  }
  
  if (!readSimulatorMap( ifp, inFileName, additionalObjectBorder,
			 fromHeight, toHeight)){
    fprintf(stderr, "ERROR: Could not read simulator file '%s'!\n",
	    inFileName);
    fclose(ifp);
    exit(-1);
  }
  else
    exit(0);

}


