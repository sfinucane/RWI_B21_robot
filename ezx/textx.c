
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/ezx/textx.c,v $
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
 * $Log: textx.c,v $
 * Revision 1.1  2002/09/14 15:27:28  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/03/11 17:11:05  tyson
 * added IR simulation and other work
 *
 * Revision 1.1.1.1  1996/09/22 16:46:35  rhino
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





#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

XFontStruct	*theFont = NULL;

/* Changes by Sebastian 94-6-10: If a font has not been freed, it will
 * be freed automatically before allocating memory for a new font. This
 * prevents a memory leak.
 */

void EZX_UseFont(GC theNewGC, char fontname[])
{
  char *tryFontName;
  char **useFontName = NULL;	/*  makes the compiler happy */
  int foundFont = 0;
  int tryingCourier = 0;

  if (theFont != NULL)
    XFreeFont(theDisplay, theFont);
  theFont = XLoadQueryFont(theDisplay, fontname);

  /* font name was not found, try to find one similar */
  /* Added 13 Oct 1994 - Tyson D Sawyer               */

  if (!theFont) {
    printf ("EZX_UseFont: Font not found - %s\n", fontname);
    tryFontName = malloc(256);
    strcpy (tryFontName+1, fontname);
    tryFontName[0] = '*';
    tryFontName[strlen(fontname)+1] = '*';
    tryFontName[strlen(fontname)+2] = 0;

    while (!foundFont) {
	printf ("EZX_UseFont: looking for font - %s\n", tryFontName);
	useFontName = XListFonts(theDisplay, tryFontName, 1, &foundFont);
	tryFontName[strlen(tryFontName)-2] = '*';
	tryFontName[strlen(tryFontName)-1] = 0;
	if (!tryingCourier && strlen(tryFontName)<5) {
	    strcpy (tryFontName, "*courier*");
	    tryingCourier = 1;
	}
    }
    printf ("EZX_UseFont: Using font - %s\n", useFontName[0]);
    theFont = XLoadQueryFont(theDisplay, useFontName[0]);
    XFreeFontNames(useFontName);
    free (tryFontName);
    if (!theFont) printf("EZX_UseFont: failed - theFont=NULL\n");
  }
  
  if (theFont != 0)
    XSetFont(theDisplay, theNewGC, theFont->fid);
}

void EZX_FreeFont(void)
{
   XFreeFont(theDisplay, theFont);
   theFont = NULL;
}




void EZX_DrawString(EZXW_p w, int x, int y, char *string)
{
   XDrawString(theDisplay, w->w, theGC, x, y, string, strlen(string));
}


void EZX_DrawText(EZXW_p w, int x, int y, char *string)
{
   XDrawImageString(theDisplay, w->w, theGC, x, y, string, strlen(string));
}


static void format_spec(int x, int y, char string[], char style, 
			int *newx, int *newy, int *width)
{
   *width = XTextWidth(theFont, string, strlen(string));
   *newx  = x;
   *newy  = y;
   switch( style ) {
        case 'l':	/* Left Alignment */
	case 'L':
	    break;
	case 'R':	/* Right Alignment */
	case 'r':
/*	    *newx = x + WSizeHints.width - *width;  */
	    *newx = x - *width;
	    break;
	case 'c':	/* Centered */ 
	case 'C':
/*	    *newx = x + (WSizeHints.width - *width) / 2; */
	    *newx = x - *width / 2;
/*	    *newy = y - EZX_GetFontHeight()/2; */
	    break;
	default:	/* use left alignment as default */
	    format_spec(x,y, string, 'L', newx,newy,width);
	 }
}


void EZX_DrawTextAt(EZXW_p w, int x, int y, char *string, char style)
{
   int	width,newx,newy;

   format_spec(x,y, string, style, &newx, &newy, &width);
   EZX_DrawText(w, newx, newy, string );
}


void EZX_DrawStringAt(EZXW_p w, int x, int y, char *string, char style)
{
   int	width,newx,newy;

   format_spec(x,y, string, style, &newx, &newy, &width);
   EZX_DrawString(w, newx, newy, string );
}


void EZX_FormatAt(EZXW_p w, int x, int y, char *string, char style,
		  int background_filled, int underlied)
{
   int	width,newx,newy;

   format_spec(x,y, string, style, &newx, &newy, &width);
   if( background_filled )
      EZX_DrawText(w, newx, newy, string );
   else
      EZX_DrawString(w, newx, newy, string );

   if( underlied )
      EZX_DrawLine(w, newx, newy+2, x+width, newy+2 );
}


int  EZX_GetFontHeight(void)
{
    if (theFont) return( theFont->ascent + theFont->descent );
    else return (0);
}


int EZX_GetTextWidth(char string[])
{
   if (theFont) return( XTextWidth(theFont, string, strlen(string)) );
   else return (0);
}

int EZX_GetCharWidth ( int which )
{
  if ( which >= (int)theFont->min_char_or_byte2 )
    return theFont->per_char[which-theFont->min_char_or_byte2].width;
  
  return 0;
}
