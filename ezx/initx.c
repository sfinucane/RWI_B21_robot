
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/ezx/initx.c,v $
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
 * $Log: initx.c,v $
 * Revision 1.1  2002/09/14 15:27:28  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/02/13 01:26:42  thrun
 * Stefan found an _old_ bug in the initialization of XWindows.
 *
 * Revision 1.2  1996/10/25 12:03:50  ws
 * Initx now includes stdlib.h and
 * XLookupString in cursorx.c now uses &theKeyEvent
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

#include <stdlib.h>

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

int	    	theScreen;		/* Screen number */
Display	       *theDisplay;		/* X server connection */
int	     	theDepth;		/* 1 if monochrome */
Colormap	theColormap;
unsigned char	theBlackPixel;
unsigned char	theWhitePixel;


static int 	theFirstTime = True;


void EZX_InitX(char *display, char *program)
{

   if (! theFirstTime) return;
   /* establish a connection to X server */
   if ( display == NULL )  display = getenv("DISPLAY");
   if ( (theDisplay = XOpenDisplay(display)) == NULL ) {
      fprintf(stderr, "\n%s: could not open display %s.\n",
	      program, XDisplayName(display));
      exit(1);
   }
   /* check for the default screen and color plane depth, etc */
   theScreen     = XDefaultScreen( theDisplay );
   theDepth      = XDefaultDepth( theDisplay, theScreen );
   theWhitePixel = XWhitePixel( theDisplay, theScreen );
   theBlackPixel = XBlackPixel( theDisplay, theScreen );
   theColormap   = XDefaultColormap( theDisplay, theScreen );

   EZX_InitDefaultColors();
   EZX_InitPatterns(8);
   theFirstTime = False;
}



void EZX_EndX(void)
{
   if (theFirstTime) return;
   XFlush( theDisplay );
   XCloseDisplay( theDisplay );
   theFirstTime = True;
}
