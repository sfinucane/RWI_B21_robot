
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/ezx/grey.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:27:28 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: grey.c,v $
 * Revision 1.1  2002/09/14 15:27:28  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:34  rhino
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





/*
 *	SGPattern.c  -	Grey Level Pattern File
 *
 *	Relevant Parameters/Variables to be changed:
 *	    PATTERN_HEIGHT :	in SGrapher.h  (#define)
 *	    PATTERN_WIDTH  :	in SGrapher.h  (#define)
 *	    mask	   :	in SGPattern.c (variable initialization)
 *
 *      uncorrelated grey - patterns    ChWi   01-13-92
 */

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"
#include <stdio.h>
#include <stdlib.h>


BitMapPoints 	theGrey[NO_GREY_LEVELS];

#ifdef old_grey_patterns

unsigned int  greyPattern[NO_GREY_LEVELS][PATTERN_HEIGHT] =
	      { {0000, 0000, 0000, 0000, 0000, 0000, 0000, 0000}, /* 00 */
		{0000, 0000, 0000, 0000, 0000, 0000, 0010, 0000},
		{0000, 0001, 0000, 0000, 0000, 0000, 0010, 0000},
		{0000, 0001, 0001, 0000, 0000, 0000, 0010, 0000},
		{0000, 0001, 0001, 0000, 0000, 0010, 0010, 0000},
		{0000, 0001, 0001, 0000, 0000, 0030, 0010, 0000},
		{0000, 0201, 0001, 0000, 0000, 0030, 0010, 0000},
		{0000, 0201 ,0201, 0000, 0000, 0030, 0010, 0000},
		{0000, 0201, 0201, 0000, 0000, 0030, 0030, 0000}, /* 08 */
		{0000, 0201, 0201, 0000, 0000, 0030, 0034, 0000},
		{0000, 0203, 0201, 0000, 0000, 0030, 0034, 0000},
		{0000, 0203, 0201, 0000, 0000, 0034, 0034, 0000},
		{0000, 0203, 0203, 0000, 0000, 0034, 0034, 0000},
		{0000, 0203, 0203, 0000, 0000, 0034, 0034, 0010},
		{0000, 0203, 0203, 0001, 0000, 0034, 0034, 0010},
		{0001, 0203, 0203, 0001, 0000, 0034, 0034, 0010},
		{0001, 0203, 0203, 0001, 0010, 0034, 0034, 0010}, /* 16 */
		{0001, 0203, 0203, 0001, 0030, 0034, 0034, 0010},
		{0201, 0203, 0203, 0001, 0030, 0034, 0034, 0010},
		{0201, 0203, 0203, 0201, 0030, 0034, 0034, 0010},
		{0201, 0203, 0203, 0201, 0030, 0034, 0034, 0030},
		{0201, 0203, 0303, 0201, 0030, 0034, 0034, 0030},
		{0201, 0203, 0303, 0201, 0030, 0074, 0034, 0030},
		{0201, 0303, 0303, 0201, 0030, 0074, 0034, 0030},
		{0201, 0303, 0303, 0201, 0030, 0074, 0074, 0030}, /* 24 */
		{0201, 0303, 0303, 0201, 0030, 0074, 0074, 0034},
		{0203, 0303, 0303, 0201, 0030, 0074, 0074, 0034},
		{0203, 0303, 0303, 0203, 0030, 0074, 0074, 0034},
		{0203, 0303, 0303, 0203, 0034, 0074, 0074, 0034},
		{0203, 0303, 0303, 0203, 0074, 0074, 0074, 0034},
		{0203, 0303, 0303, 0303, 0074, 0074, 0074, 0034},
		{0303, 0303, 0303, 0303, 0074, 0074, 0074, 0034},
		{0303, 0303, 0303, 0303, 0074, 0074, 0074, 0074}, /* 32 */
		{0307, 0303, 0303, 0303, 0074, 0074, 0074, 0074},
		{0307, 0303, 0303, 0303, 0074, 0074, 0074, 0076},
		{0307, 0303, 0303, 0303, 0076, 0074, 0074, 0076},
		{0307, 0303, 0303, 0307, 0076, 0074, 0074, 0076},
		{0307, 0303, 0303, 0347, 0076, 0074, 0074, 0076},
		{0307, 0303, 0303, 0347, 0176, 0074, 0074, 0076},
		{0307, 0303, 0303, 0347, 0176, 0074, 0074, 0176},
		{0347, 0303, 0303, 0347, 0176, 0074, 0074, 0176}, /* 40 */
		{0347, 0303, 0303, 0347, 0176, 0074, 0076, 0176},
		{0347, 0307, 0303, 0347, 0176, 0074, 0076, 0176},
		{0347, 0307, 0303, 0347, 0176, 0076, 0076, 0176},
		{0347, 0307, 0307, 0347, 0176, 0076, 0076, 0176},
		{0357, 0307, 0307, 0347, 0176, 0076, 0076, 0176},
		{0357, 0307, 0307, 0347, 0177, 0076, 0076, 0176},
		{0357, 0307, 0307, 0347, 0177, 0076, 0076, 0177},
		{0357, 0307, 0307, 0367, 0177, 0076, 0076, 0177}, /* 48 */
		{0357, 0307, 0307, 0377, 0177, 0076, 0076, 0177},
		{0357, 0307, 0307, 0377, 0177, 0076, 0076, 0377},
		{0357, 0307, 0307, 0377, 0377, 0076, 0076, 0377},
		{0377, 0307, 0307, 0377, 0377, 0076, 0076, 0377},
		{0377, 0307, 0347, 0377, 0377, 0076, 0076, 0377},
		{0377, 0307, 0347, 0377, 0377, 0176, 0076, 0377},
		{0377, 0347, 0347, 0377, 0377, 0176, 0076, 0377},
		{0377, 0347, 0347, 0377, 0377, 0176, 0176, 0377}, /* 56 */
		{0377, 0357, 0347, 0377, 0377, 0176, 0176, 0377},
		{0377, 0357, 0347, 0377, 0377, 0176, 0177, 0377},
		{0377, 0357, 0347, 0377, 0377, 0177, 0177, 0377},
		{0377, 0357, 0357, 0377, 0377, 0177, 0177, 0377},
		{0377, 0357, 0377, 0377, 0377, 0177, 0177, 0377},
		{0377, 0357, 0377, 0377, 0377, 0177, 0377, 0377},
		{0377, 0357, 0377, 0377, 0377, 0377, 0377, 0377},
		{0377, 0377, 0377, 0377, 0377, 0377, 0377, 0377}  /* 64 */
	     };


#else

unsigned int  greyPattern[NO_GREY_LEVELS][PATTERN_HEIGHT] =
 {  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x01,0x00,0x00,0x00,0x10,0x00,0x00,0x00},
    {0x11,0x00,0x00,0x00,0x10,0x00,0x00,0x00},
    {0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00},
    {0x11,0x00,0x04,0x00,0x11,0x00,0x00,0x00},
    {0x11,0x00,0x04,0x00,0x11,0x00,0x40,0x00},
    {0x11,0x00,0x44,0x00,0x11,0x00,0x40,0x00},
    {0x11,0x00,0x44,0x00,0x11,0x00,0x44,0x00},
    {0x15,0x00,0x44,0x00,0x11,0x00,0x44,0x00},
    {0x15,0x00,0x44,0x00,0x51,0x00,0x44,0x00},
    {0x55,0x00,0x44,0x00,0x51,0x00,0x44,0x00},
    {0x55,0x00,0x44,0x00,0x55,0x00,0x44,0x00},
    {0x55,0x00,0x45,0x00,0x55,0x00,0x44,0x00},
    {0x55,0x00,0x45,0x00,0x55,0x00,0x54,0x00},
    {0x55,0x00,0x55,0x00,0x55,0x00,0x54,0x00},
    {0x55,0x00,0x55,0x00,0x55,0x00,0x55,0x00},
    {0x55,0x02,0x55,0x00,0x55,0x00,0x55,0x00},
    {0x55,0x02,0x55,0x00,0x55,0x20,0x55,0x00},
    {0x55,0x22,0x55,0x00,0x55,0x20,0x55,0x00},
    {0x55,0x22,0x55,0x00,0x55,0x22,0x55,0x00},
    {0x55,0x22,0x55,0x08,0x55,0x22,0x55,0x00},
    {0x55,0x22,0x55,0x08,0x55,0x22,0x55,0x80},
    {0x55,0x22,0x55,0x88,0x55,0x22,0x55,0x80},
    {0x55,0x22,0x55,0x88,0x55,0x22,0x55,0x88},
    {0x55,0x2a,0x55,0x88,0x55,0x22,0x55,0x88},
    {0x55,0x2a,0x55,0x88,0x55,0xa2,0x55,0x88},
    {0x55,0xaa,0x55,0x88,0x55,0xa2,0x55,0x88},
    {0x55,0xaa,0x55,0x88,0x55,0xaa,0x55,0x88},
    {0x55,0xaa,0x55,0x8a,0x55,0xaa,0x55,0x88},
    {0x55,0xaa,0x55,0x8a,0x55,0xaa,0x55,0xa8},
    {0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xa8},
    {0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa},
    {0x57,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa},
    {0x57,0xaa,0x55,0xaa,0x75,0xaa,0x55,0xaa},
    {0x77,0xaa,0x55,0xaa,0x75,0xaa,0x55,0xaa},
    {0x77,0xaa,0x55,0xaa,0x77,0xaa,0x55,0xaa},
    {0x77,0xaa,0x5d,0xaa,0x77,0xaa,0x55,0xaa},
    {0x77,0xaa,0x5d,0xaa,0x77,0xaa,0xd5,0xaa},
    {0x77,0xaa,0xdd,0xaa,0x77,0xaa,0xd5,0xaa},
    {0x77,0xaa,0xdd,0xaa,0x77,0xaa,0xdd,0xaa},
    {0x7f,0xaa,0xdd,0xaa,0x77,0xaa,0xdd,0xaa},
    {0x7f,0xaa,0xdd,0xaa,0xf7,0xaa,0xdd,0xaa},
    {0xff,0xaa,0xdd,0xaa,0xf7,0xaa,0xdd,0xaa},
    {0xff,0xaa,0xdd,0xaa,0xff,0xaa,0xdd,0xaa},
    {0xff,0xaa,0xdf,0xaa,0xff,0xaa,0xdd,0xaa},
    {0xff,0xaa,0xdf,0xaa,0xff,0xaa,0xfd,0xaa},
    {0xff,0xaa,0xff,0xaa,0xff,0xaa,0xfd,0xaa},
    {0xff,0xaa,0xff,0xaa,0xff,0xaa,0xff,0xaa},
    {0xff,0xab,0xff,0xaa,0xff,0xaa,0xff,0xaa},
    {0xff,0xab,0xff,0xaa,0xff,0xba,0xff,0xaa},
    {0xff,0xbb,0xff,0xaa,0xff,0xba,0xff,0xaa},
    {0xff,0xbb,0xff,0xaa,0xff,0xbb,0xff,0xaa},
    {0xff,0xbb,0xff,0xae,0xff,0xbb,0xff,0xaa},
    {0xff,0xbb,0xff,0xae,0xff,0xbb,0xff,0xea},
    {0xff,0xbb,0xff,0xee,0xff,0xbb,0xff,0xea},
    {0xff,0xbb,0xff,0xee,0xff,0xbb,0xff,0xee},
    {0xff,0xbf,0xff,0xee,0xff,0xbb,0xff,0xee},
    {0xff,0xbf,0xff,0xee,0xff,0xfb,0xff,0xee},
    {0xff,0xff,0xff,0xee,0xff,0xfb,0xff,0xee},
    {0xff,0xff,0xff,0xee,0xff,0xff,0xff,0xee},
    {0xff,0xff,0xff,0xef,0xff,0xff,0xff,0xee},
    {0xff,0xff,0xff,0xef,0xff,0xff,0xff,0xfe},
    {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe},
    {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}};

#endif

unsigned int	masks[PATTERN_WIDTH] =
		 { 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01 };


static int pattern_width = 0, pattern_height = 0;




void EZX_InitPatterns(int scale)	/* scale: typically: 8 */
{
   int i;
   for( i = 0; i < NO_GREY_LEVELS; i++ )
      EZX_CreatePattern( &greyPattern[i][0], &theGrey[i], scale);
}

void EZX_CreatePattern(unsigned int pattern[PATTERN_HEIGHT], 
		       BitMapPoints *mapPoints, int scale)
{
   int	i, j, count=0, x=0, y=0;
   
   /* these two lines are new, as is "scale". S. Thrun, Aug-12-93 */
   pattern_width  = scale;
   pattern_height = (scale * PATTERN_HEIGHT) / PATTERN_WIDTH;
 
   mapPoints->points =
      (XPoint *) malloc(sizeof(XPoint) * (pattern_width * pattern_height));
   if( mapPoints->points == NULL ) {
      fprintf( stderr, "\nCreatePattern: malloc error, NULL returned" );
      exit(1);
   }

   for( i = 0; i < pattern_height; i++ )
      for( j = 0; j < pattern_width; j++ )
	 if( pattern[i % PATTERN_HEIGHT] & masks[j] ) {
	    if( count == 0 ) {
	       mapPoints->start.x = pattern_width - 1 - j;
	       mapPoints->start.y = i;
	    }
	    else {
	       /*... points are in relative coordinate of previous pt ...*/
	       mapPoints->points[count].x = pattern_width - 1 - j - x;
	       mapPoints->points[count].y = i - y;
	    }
	    x = pattern_width - 1 - j;  y = i;
	    count++;
	 }
   mapPoints->npoints = count;
}


void EZX_DrawPattern(EZXW_p w, int x, int y, BitMapPoints *pattern)
{
   EZX_ClearRectangle(w, x, y, pattern_width, pattern_height );
   pattern->points[0].x = x + pattern->start.x;
   pattern->points[0].y = y + pattern->start.y;
   EZX_DrawPoints(w, pattern->npoints, pattern->points );
}


void EZX_DrawGrey(EZXW_p w, int x, int y, int g)
{
   g = (g < 0) ? 0 : ((g >= NO_GREY_LEVELS) ? NO_GREY_LEVELS-1 : g);
   EZX_DrawPattern(w, x, y, &theGrey[g]);
}


