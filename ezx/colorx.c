
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/ezx/colorx.c,v $
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
 * $Log: colorx.c,v $
 * Revision 1.1  2002/09/14 15:27:28  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1999/04/18 19:00:07  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
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
 *  colorx.c	-   set up colors
 *  Created by Long-Ji Lin (ljl) at Aug 6, 1991.
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

static EZX_color = C_BLACK;

unsigned long	thePixels[MAXCOLORS];


static char	*theColorNames[] = {
   "white", "black", "red", "seagreen2", "blue", 
   "yellow", "gold", "violet", "pink",
   "grey0", "grey5",
   "grey10", "grey15", 
   "grey20", "grey25", 
   "grey30", "grey35", 
   "grey40", "grey45", 
   "grey50", "grey55", 
   "grey60", "grey65", 
   "grey70", "grey75", 
   "grey80", "grey85", 
   "grey90", "grey95", "grey100",
   "mediumvioletred", "mediumpurple3", "palegreen4", "cyan", "steelblue4",
   "orangered4", "khaki4", "darkturquoise", "forestgreen", "oldlace",
   "lightslategrey", "slategrey", "darkslategrey", "paleturquoise4",
   "limegreen", "cadetblue", "deeppink", "magenta2", "sienna4", 
   "pink1", "turquoise4", "royalblue", "yellow2", "#bdc1ff",

   /* 101 grey values for nicer maps */

   "grey0", "grey1", "grey2", "grey3", "grey4", "grey5", "grey6",
   "grey7", "grey8", "grey9", "grey10", "grey11", "grey12", "grey13",
   "grey14", "grey15", "grey16", "grey17", "grey18", "grey19", "grey20",
   "grey21", "grey22", "grey23", "grey24", "grey25", "grey26", "grey27",
   "grey28", "grey29", "grey30", "grey31", "grey32", "grey33", "grey34",
   "grey35", "grey36", "grey37", "grey38", "grey39", "grey40", "grey41",
   "grey42", "grey43", "grey44", "grey45", "grey46", "grey47", "grey48",
   "grey49", "grey50", "grey51", "grey52", "grey53", "grey54", "grey55",
   "grey56", "grey57", "grey58", "grey59", "grey60", "grey61", "grey62",
   "grey63", "grey64", "grey65", "grey66", "grey67", "grey68", "grey69",
   "grey70", "grey71", "grey72", "grey73", "grey74", "grey75", "grey76",
   "grey77", "grey78", "grey79", "grey79", "grey81", "grey82", "grey83",
   "grey84", "grey85", "grey86", "grey87", "grey88", "grey89", "grey90",
   "grey91", "grey92", "grey93", "grey94", "grey95", "grey96", "grey97",
   "grey98", "grey99", "grey100",


   "white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white"  
   };


void EZX_InitDefaultColors(void)
{
   XColor	theRGBColor, theHardwareColor;
   int		theStatus;
   int		i;
   
   if (theDepth==1) {
      /* monochrome system */
      thePixels[C_WHITE] = theWhitePixel;
      thePixels[C_BLACK] = theBlackPixel;
   } else {
      for(i=0;i<MAXCOLORS;++i) {
	 theStatus = XLookupColor(theDisplay, theColormap, theColorNames[i],
				  &theRGBColor, &theHardwareColor);
	 if (theStatus != 0) {
	    theStatus = XAllocColor(theDisplay, theColormap, 
				    &theHardwareColor);
	    if (theStatus != 0)
	       thePixels[i] = theHardwareColor.pixel;
	    else
	       thePixels[i] = theBlackPixel;
	 }
      }
   }
}



int  EZX_SetColor(int color)			/* set foreground color */
{
   int save = EZX_color;
   if (color >= MAXCOLORS || color < 0) {
      fprintf(stderr, "Wrong color: %d\n", color);
      return save;
   }
   XSetForeground(theDisplay, theGC, thePixels[EZX_color = color]);
   return save;
}


int EZX_RequestColor ( int color )
{
  return thePixels[color];
}


int EZX_SetBackgroundColor ( int color )
{
   int save = EZX_color;
   if (color >= MAXCOLORS || color < 0) {
      printf("Wrong color: %d\n", color);
      return save;
   }
  
   XSetBackground(theDisplay, theGC, thePixels[EZX_color = color]);
/*     fprintf (stderr,"\nHere is no .."); */
   return save;
}
  

int EZX_GetColor(void)
{
   return EZX_color;
}

