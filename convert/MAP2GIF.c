
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/MAP2GIF.c,v $
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
 * $Log: MAP2GIF.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1999/12/01 16:14:48  haehnel
 * *** empty log message ***
 *
 * Revision 1.3  1998/11/16 18:48:57  wolfram
 * Minor changes
 *
 * Revision 1.2  1998/03/27 22:20:37  wolfram
 * added clipping option, changed the parameters
 *
 * Revision 1.1  1997/05/10 17:06:02  wolfram
 * MAP2GIF converts RHINO map files to GIF files.
 *
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



#define NUMBER_OF_GREY_LEVELS 100
#define MAX_STRING_LENGTH 10000
#define BW_THRESHOLD 0.8

int
main( int argc, char *argv[] )
{
  FILE *ifp, *ofp;
   
  register int i, j;
  int sizeX, sizeY, clip = 0, bw = 0;
  register int gifSizeX, gifSizeY;
  int zoom = 1;
  int minX, maxX, minY, maxY;
  
  gdImagePtr GifPic;
  int color[NUMBER_OF_GREY_LEVELS];
  int unknownColor;

  char line[MAX_STRING_LENGTH];
  
  char* inFileName;
  char* outFileName;
   
  int argCount = 0;

#define USAGE_STRING "usage: %s mapfile [-bw] [-zoom z] [-clip] giffile\n"
  
  if (argc < 3 || argc > 7)
    {
      fprintf(stderr,
	      USAGE_STRING,
	      argv[0]);
      exit(0);
    }      
  
  argCount++;
  inFileName = argv[argCount];
  if ((ifp = fopen(inFileName,"r")) == NULL) {
    fprintf(stderr,"ERROR: Could not open map file '%s'!\n",
	    inFileName);
    exit(0);
  }
  argCount++;

  if (argCount < argc && strcmp(argv[argCount], "-zoom") == 0){
    argCount++;
    if (argCount < argc){
      zoom = atoi(argv[argCount]);
      if ( zoom < 1 || zoom > 100){
	fprintf(stderr,"ERROR: zoom factor must be between 1 and 100, using 1!\n");
	zoom = 1;
      }
      argCount++;
    }
    else{
      fprintf(stderr,
	      USAGE_STRING,
	      argv[0]);
      exit(0);
    }
    
  }

  if (argCount < argc && strcmp(argv[argCount], "-clip") == 0){
    clip = 1;
    argCount++;
  }

  if (argCount < argc && strcmp(argv[argCount], "-bw") == 0){
    bw = 1;
    argCount++;
  }

  if (argCount < argc){
    outFileName = argv[argCount];
    if ((ofp = fopen(outFileName,"wt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open gif file '%s'!\n",outFileName);
      exit(0);
    }
  }
  else{
    fprintf(stderr,
	    USAGE_STRING,
	    argv[0]);
    exit(0);
  }
  

  /* seek header for map data in rhino-Type file */

  while ((fgets(line,MAX_STRING_LENGTH,ifp) != NULL)
	 && (strncmp("global_map[0]", line , 13) != 0));

  if (feof(ifp) || sscanf(line,"global_map[0]: %d %d",&sizeY, &sizeX) != 2) {
    fprintf(stderr,"# ERROR: corrupted file %s\n",inFileName);
    exit(0);
  }

  minX = 0;
  minY = 0;
  maxX = sizeX;
  maxY = sizeY;

  if (clip){
    FILE *saveP = ifp;
    float temp;
    minX = sizeX;
    minY = sizeY;
    maxX = 0;
    maxY = 0;
    for (i=0; i < sizeX; i++)
      for (j=0; j < sizeY; j++) {
	if (fscanf(ifp,"%e",&temp) == 1){
	  if (temp >=  0.0 && temp <= 1.0){
	    if (i < minX) minX = i;
	    if (i > maxX) maxX = i;
	    if (j < minY) minY = j;
	    if (j > maxY) maxY = j;
	  }
	}
	
	else{
	  fprintf(stderr, "# ERROR map file too short!\n");
	  exit(0);
	}
      }
    rewind(ifp);
    while ((fgets(line,MAX_STRING_LENGTH,ifp) != NULL)
	   && (strncmp("global_map[0]", line , 13) != 0));
    maxX++;
    maxY++;
  }

  gifSizeX = (maxX-minX) * zoom;
  gifSizeY = (maxY-minY) * zoom;
  GifPic = gdImageCreate( gifSizeX, gifSizeY);
  
  for (i = 0; i < NUMBER_OF_GREY_LEVELS; i++){
    int greyValue = i * (255.0 / NUMBER_OF_GREY_LEVELS) + 0.5;
    color[i] = gdImageColorAllocate( GifPic, greyValue, greyValue, greyValue );
  }
  unknownColor = gdImageColorAllocate( GifPic, 78, 170, 68);


  for (i=0; i < sizeX; i++)
    for (j=0; j < sizeY; j++) {
      register int x, y;
      int tmpX, tmpY;
      float temp;
      int col=0;

      if (fscanf(ifp,"%e",&temp) == 1){

	if (i >= minX && i < maxX && j >= minY && j < maxY){
	  if (bw){
	    if (temp < BW_THRESHOLD)
	      col = color[0];                        /* black */
	    else
	      col = color[NUMBER_OF_GREY_LEVELS-1];  /* white */
	  } else {
	    if (temp >=  0.0 && temp <= 1.0)
	      col = color[(int) (temp * (NUMBER_OF_GREY_LEVELS-1) + 0.5)];
	    else
	      col = unknownColor;
	  }
	  tmpX = (i-minX)*zoom;
	  tmpY = (j-minY)*zoom;
	  for (x = tmpX; x < tmpX + zoom; x++)
	    for (y = tmpY; y < tmpY + zoom; y++)
	      gdImageSetPixel( GifPic, x, gifSizeY - y - 1, col );
	}
      }
      else{
	fprintf(stderr, "# ERROR map file too short!\n");
	exit(0);
      }
    }
  
  gdImageGif( GifPic, ofp );
  fclose(ifp);
  fclose(ofp);
  exit(0);
} 

